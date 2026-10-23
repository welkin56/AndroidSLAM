package com.example.slamcapture

import android.Manifest
import android.content.pm.PackageManager
import android.content.res.ColorStateList
import android.graphics.Color
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.os.Looper
import android.os.SystemClock
import android.util.Log
import android.view.MotionEvent
import android.view.View
import android.widget.Button
import android.widget.CheckBox
import android.widget.EditText
import android.widget.FrameLayout
import android.widget.LinearLayout
import android.widget.SeekBar
import android.widget.TextView
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.SwitchCompat
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.core.view.GravityCompat
import androidx.drawerlayout.widget.DrawerLayout
import java.io.File
import java.io.IOException
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import kotlin.math.abs
import kotlin.math.roundToInt

/**
 * SLAM 主界面。**跑 SLAM 的那一屏。**
 *
 * 排版和录制页**故意不同**：这里是「状态 → 画面 → 按钮」，录制页是「画面 → 状态 → 按钮」。
 * 原因：这一页你盯着的是「**现在能不能用**」（词袋加载完没有、跟踪丢没丢），画面反而是辅助。
 *
 * 相机和 IMU **这一页自己持有**，和 [CaptureActivity] 各开各的 —— 两个模式互斥，
 * 理由是对焦所有权和性能（APP_DESIGN §6.3）。
 *
 * 那个主按钮是**三段**（和录制页的两段式对齐）：
 *
 * ```
 * 未准备 ──[准备 SLAM]──► 加载中（词袋 38 s）──► 就绪
 *                                                │
 *                                      [启动 SLAM] ▼
 *                                              运行中 ──[结束]──► 就绪
 * ```
 *
 * **准备** = 构造 `System`（贵且慢，38 秒）；**启动** = 开相机 + IMU + 开始吃帧。
 * 分开的理由和录制页那个一样：**贵的那一步不该被反复付**。
 *
 * ⚠️ **没有「取消准备」** —— `System` 构造没法取消，而且造好之后也没有安全的销毁时机
 * （见 [onButtonSlam] 上面那段）。所以这个按钮是**单行道**。
 */
class MainActivity : AppCompatActivity() {

    companion object {
        private const val TAG = "MainActivity"

        /** 入口页选中的已保存地图（绝对路径）。**没有这个 extra 就是「新建地图」**。 */
        const val EXTRA_MAP_FILE = "map_file"

        private const val VOCAB_NAME = "ORBvoc.txt"
        private const val SETTINGS_NAME = "phone.yaml"

        // SlamBridge.nativeState() 的取值，和 C++ 侧 slam::State 一一对应。
        private const val SLAM_IDLE = 0
        private const val SLAM_LOADING = 1
        private const val SLAM_READY = 2
        private const val SLAM_FAILED = 3

        // ORB-SLAM3 的 eTrackingState（Tracking.h:121）。
        private const val TRK_NOT_READY = -1
        private const val TRK_NO_IMAGES = 0
        private const val TRK_NOT_INITIALIZED = 1
        private const val TRK_OK = 2
        private const val TRK_RECENTLY_LOST = 3
        private const val TRK_LOST = 4

        private const val TICK_MS = 250L
        private const val REQ_CAMERA = 1

        /** 预热时长：相机帧硬按住这么久，让 IMU 先写满。和录制模式同值。 */
        private const val PREROLL_NS = 300_000_000L

        /** 两个开关的**记忆**。抽屉里那两个，下次进来还是上次的状态。 */
        private const val PREF = "slamapp"
        private const val PREF_RECORD = "record"
        private const val PREF_MEASURE = "measure"
        private const val PREF_MAP = "map"
        private const val PREF_MAP_RANGE = "mapRange"
        /** 小地图缩放档位（`MapView.ZOOM_*` 的下标）。 */
        private const val PREF_MAP_ZOOM = "mapZoom"
        private const val PREF_WAYPOINTS = "waypoints"
        private const val PREF_HISTORY = "historyOn"
        /** 逐条隐藏的历史轨迹：绝对路径集合。 */
        private const val PREF_HIST_HIDDEN = "historyHidden"

        /** 历史轨迹每条最多画这么多点（文件可能几万行，stride 抽稀）。 */
        private const val kHistoryMaxPts = 2000

        /** `nativeDisplayTransforms` 的缓冲：够放一条很长的合并链。 */
        private const val kDisplayTfMax = 32

        /**
         * 点云的**全量上限**，和 `slam_engine.cpp` 的 `kMapPointsMax` 一一对应。
         * 那边超了会 stride 抽稀，所以这一侧是**固定缓冲、别扩容**（和 `trajBuf` 同款契约）。
         */
        private const val kMapPointsMax = 5000

        /** 厚度滑块的刻度数。`SeekBar` 只有整数，所以在 `0..100` 上线性映射到米。 */
        private const val kRangeSeekMax = 100

        /** 按住缩放键时，**每隔多久走一档**。大约一秒走十档。 */
        private const val kZoomRepeatMs = 95L

        private val DARK_TEXT = Color.parseColor("#DE000000")
        private val RED = Color.parseColor("#C62828")

        /**
         * 「加点」吸到**已有点**的半径（图像像素）。
         *
         * ⚠️ 和 `measure_center.cpp` 的 `kSnapRadiusPx` 同值纯属巧合，**是另一个常数**
         * （那个吸地图点，这个吸标记点）。**手感参数，跑完再定。**
         */
        private const val kPointSnapPx = 20f

        /** 折线读回来的缓冲**至少留这么多点的余量**。见 [readPinned] 里那段。 */
        private const val kPinnedSlack = 8

        /** `nativeGetPinned` 每点第 3 个 float 的位。**和 `slam_engine.cpp` 那侧一一对应。** */
        private const val kFlagFront = 1   // 在相机前方
        private const val kFlagClosed = 2  // 它之后「断开」了
        private const val kFlagLow = 4     // 不太准（平面兜底/勉强）⟹ 画空心圈

        /** 路径点 flags 的位。**和 `System::GetWaypoints` 一一对应。** */
        private const val kFlagWpValid = 1L
        private const val kFlagWpCurrent = 2L
    }

    private lateinit var btnSlam: Button
    private lateinit var swRecord: SwitchCompat
    private lateinit var swMeasure: SwitchCompat
    private lateinit var tvStatus: TextView
    private lateinit var preview: PreviewView
    private lateinit var drawer: DrawerLayout

    private val uiHandler = Handler(Looper.getMainLooper())

    /** 两个开关的记忆。是本 App 唯一用 SharedPreferences 的地方。 */
    private val pref by lazy { getSharedPreferences(PREF, MODE_PRIVATE) }

    // ---------------- 页面状态 ----------------

    /** 入口页选中的已保存地图。**null = 新建地图**。不加 enum，它本身就是那个参数。 */
    private var mapFile: String? = null

    /** C++ 那边的 `System` 状态。**只在主线程写**。 */
    private var slamState = SLAM_IDLE
    private var slamElapsedMs = 0L

    /**
     * 「正在吃帧」—— 相机和 IMU 开着、`nativeStart()` 调过了。
     *
     * 和 [slamState] 是**两件事**：`System` 造好之后一直活着（`slamState == SLAM_READY`），
     * 而吃不吃帧由用户按按钮决定。这正是「停止再启动不用重等 38 秒」的原因。
     */
    private var slamRunning = false

    // ---------------- 硬件（本页自己持有）----------------

    private lateinit var camThread: HandlerThread
    private lateinit var cam: CameraCapture
    private var imu: ImuCollector? = null
    private var imuThread: HandlerThread? = null

    /** 预热闸：这个时刻之前的相机帧不喂也不录。相机线程读、主线程写。 */
    @Volatile
    private var framesFromTsNs = 0L

    // ---------------- 落盘（抽屉里的开关）----------------

    /** 只在录制中非 null。相机线程和 IMU 线程都会读，必须 volatile。 */
    @Volatile
    private var writer: SessionWriter? = null
    private var recording = false

    /**
     * 这次**会话**的目录：`files/maps/<时间戳>/`。
     *
     * ⚠️ **一次「新建地图」= 一个目录**，第一次按「启动 SLAM」时建，之后一直复用 ——
     * 否则「结束再启动」会把同一次新建地图裂成好几个文件夹（2026-09-29 用户报的）。
     */
    private var sessionDir: File? = null

    /** atlas 的账：`[图数, 关键帧数, 地图点数]`。复用同一个数组。 */
    private val atlasBuf = IntArray(3)

    /** 最近一次存图给的文件名（不含 `.osa`），成功后显示给用户。 */
    private var savedName: String? = null

    /** 存图状态那行字。`render()` 里按 C++ 的 `nativeSaveState()` 更新。 */
    private var saveMsg: String? = null

    /** 正在存图。**ticker 靠它才继续跑** —— 存图发生在刚停止之后，那时它本来要停了。 */
    private var saving = false

    /**
     * 正在把 atlas 恢复成「上一次保存」的样子。
     *
     * 恢复期间**不读 atlas / 轨迹** —— C++ 那头正在重建它们，`Atlas::PostLoad()` 没有锁。
     * ticker 也靠它继续跑：要轮询到「完成」才能把按钮放开。
     */
    private var restoring = false

    /** 上一次看到的恢复状态。只在它变化时改 [restoreMsg]（否则每帧都重写同一句）。 */
    private var restoreStateSeen = 0

    /** 恢复那行字。 */
    private var restoreMsg: String? = null

    /** 最近一次请求是「入口页换源」而不是「这一趟没保存」。只影响提示语。 */
    private var atlasSwitching = false

    /** 本趟的时间戳 —— 轨迹和录像都用它，一趟固定（中途开录制也不换目录）。 */
    private var runStamp = ""

    /** 本趟的录像目录 `<会话>/rec/<runStamp>/`。没录过就是 null。 */
    private var curRecDir: File? = null

    /**
     * 本趟轨迹的前缀（含目录、**不含扩展名**）：`<会话>/traj/traj-<runStamp>`。
     *
     * 先落在 `traj/`，**存图成功那一刻**才搬进 `map-<id>/`（见 [applyPendingSave]）。
     * 没存下的图不该在磁盘上留下文件夹 —— 否则回滚后 map id 会被下一张图复用。
     */
    private var curTrajBase: File? = null

    /** 本趟轨迹所属的图。`nativeStop()` 之后立刻快照；-1 = 没有。 */
    private var runMapId = -1L

    /** 本趟的合并链，每条 10 个 float（见 [SlamBridge.nativeLastMerges]）。 */
    private var runMerges: List<FloatArray> = emptyList()

    /** 上一次保留的目录，只给状态栏显示。 */
    private var lastRunDir: File? = null

    /**
     * 存图请求发出时的归档上下文。
     *
     * ⚠️ 存图是异步的，完成时用户可能已经开了**下一趟** —— 那时 `runMapId` /
     * `curTrajBase` 都已经被重置成新趟的值了。归档要用请求那一刻的快照。
     */
    private class PendingArchive(
        val session: File,
        val mapId: Long,
        val merges: List<FloatArray>,
        val base: File?
    )
    private var pendingArchive: PendingArchive? = null

    // ---------------- 位姿 ----------------

    /** 复用同一个数组 —— JNI 里构造 Java 数组代价高（§6 约定 3）。 */
    private val poseBuf = FloatArray(16)
    private var poseValid = false
    private var trkState = TRK_NOT_READY

    // ---------------- 测距（画面中心）----------------

    /** `[0]` 距离m、`[1..3]` 目标世界坐标、`[4]`/`[5]` 目标图像像素坐标。同样复用。 */
    private val measureBuf = FloatArray(7)

    /** 给 [PreviewView.Overlay] 用的状态，比 C++ 那三档多一个「跟踪未就绪」。 */
    private var measureState = PreviewView.MeasureState.NO_TRACK

    // ---------------- 多点折线（绑在 swMeasure 那个开关上）----------------

    private lateinit var rowMeasure: LinearLayout
    private lateinit var btnAddPoint: Button
    private lateinit var btnBreakChain: Button
    private lateinit var btnUndoPoint: Button
    private lateinit var btnResetPoints: Button

    /**
     * 折线**整个存在 C++ 侧** —— 它要存 `MapPoint*`（世界坐标会变，身份不会）。
     * 这一侧只留一个**读回来的缓冲**，照着画：刷新 / 投影 / 算长度全在那边一次给完。
     *
     * 布局：每点 3 个 `px py flags`；每段 5 个 `x0 y0 x1 y1 len`；最后 5 个是橡皮筋。
     */
    private var pinnedBuf = FloatArray(8 * 8 + 1)

    /** 上一次读回来几个点。**只信这个，别拿 `pinnedBuf.size` 当点数**（那是容量）。 */
    private var pinnedN = 0

    /** 当前这条折线还开着吗 —— 开着才拉橡皮筋，也才让「断开」可点。 */
    private val chainOpen: Boolean
        get() = pinnedN > 0 && (pinnedBuf[1 + (pinnedN - 1) * 3 + 2].toInt() and kFlagClosed) == 0

    /** 第 i 段那 5 个数在缓冲里的起点。 */
    private fun segAt(i: Int) = 1 + 3 * pinnedN + 5 * i

    /** 段标签缓存：上次的值 + 上次的文字。见 [segLabel]。 */
    private val segLast = ArrayList<Float>()
    private val segLabels = ArrayList<String>()

    // ---------------- 2D 地图小窗 ----------------

    private lateinit var mapView: MapView

    /**
     * 给 [MapView] 的轨迹缓冲，复用。`3 × kTrajExportMax`。
     *
     * ⚠️ **不用扩容取全** —— 那侧是 stride 抽稀的**形状**契约，和 `pinnedBuf` 正好相反。
     */
    private val trajBuf = FloatArray(3 * 512)

    /**
     * 给 [MapView] 的点云缓冲，复用。`3 × kMapPointsMax`。
     *
     * 那边写的是**快照**（SLAM 线程每秒重建一次），这一侧只是 memcpy —— 契约和 [trajBuf] 一样：
     * **固定缓冲、别扩容**。
     */
    private val mapPtBuf = FloatArray(3 * kMapPointsMax)

    /** 小地图那一整块。开关直接作用于它（连角上那两个缩放键一起藏）。 */
    private lateinit var mapWrap: FrameLayout
    private lateinit var swMap: SwitchCompat
    private lateinit var sbMapRange: SeekBar
    private lateinit var tvMapRange: TextView
    private lateinit var btnMapZoomIn: Button
    private lateinit var btnMapZoomOut: Button

    /** 上一次见到的「地图重置次数」。涨了 ⟹ 世界系换过 ⟹ 老点全废。 */
    private var resetCountSeen = 0

    /** 缩放键按住时的连发定时器。**挂在 [uiHandler] 上** —— `onDestroy` 那句
     *  `removeCallbacksAndMessages(null)` 顺手就把它清了。 */
    private var zoomRepeat: Runnable? = null

    /** 上一次落盘的缩放档位。长按连按时只在**真的换档**时才写盘。 */
    private var zoomSaved = -1

    // ---------------- 路径点 ----------------
    private lateinit var swWaypoints: SwitchCompat
    private lateinit var btnWpAdd: Button
    private lateinit var btnWpManage: Button

    /** 路径点缓冲。`wpMeta` 每点 4 个：`id, mapId, kfId, flags`。 */
    private var wpPos = FloatArray(3 * 16)
    private var wpMeta = LongArray(4 * 16)
    private var wpNames = arrayOfNulls<String>(16)
    private var wpN = 0

    /** 上次读到的那一版。只有它变了才重建小地图那一层。 */
    private var wpRevisionSeen = -1L

    /** 上一次加路径点的结果（`nativeWaypointState`），用来只提示一次。 */
    private var wpStateSeen = 0

    /** 加路径点失败时给人看的那句话。 */
    private var wpMsg: String? = null

    // ---------------- 历史轨迹（小地图上的蓝色层）----------------

    private lateinit var swHistory: SwitchCompat
    private lateinit var btnHistoryManage: Button

    /** `nativeDisplayTransforms` 的缓冲：每条 9 个 float（`id s qx qy qz qw tx ty tz`）。 */
    private val histTfBuf = FloatArray(9 * kDisplayTfMax)

    /** 读一条轨迹的缓冲：`3 × kHistoryMaxPts`。 */
    private val histPtsBuf = FloatArray(3 * kHistoryMaxPts)

    /** 当前显示图里存在的全部轨迹（含隐藏的）。 */
    private var histEntries: List<TraceEntry> = emptyList()

    /** 上次重建用的上下文（会话路径 + displayId + 所有变换）。没变就不碰磁盘。 */
    private var histCtx = ""

    /** 下次一定要重建：开关 / 增删 / 存图归档 / 换图。 */
    private var histDirty = true

    /** 真正画出来的条数 —— [refreshMap] 的 hint 要用。 */
    private var histShown = 0

    /** 存在的条数 —— 管理按钮的可用状态。 */
    private var histAll = 0

    /** 隐藏的轨迹（绝对路径），和 pref 里的 StringSet 同步。 */
    private var histHidden: MutableSet<String> = mutableSetOf()

    /** `map-<id>/traj-*.txt` + 那个文件夹到当前坐标系的累计变换（9 个 float）。 */
    private class TraceEntry(val file: File, val mapId: Long, val tf: FloatArray)

    // ---------------- 生命周期 ----------------

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        btnSlam = findViewById(R.id.btnSlam)
        swRecord = findViewById(R.id.swRecord)
        swMeasure = findViewById(R.id.swMeasure)
        tvStatus = findViewById(R.id.tvStatus)
        preview = findViewById(R.id.preview)
        mapView = findViewById(R.id.mapView)
        mapWrap = findViewById(R.id.mapWrap)
        swMap = findViewById(R.id.swMap)
        sbMapRange = findViewById(R.id.sbMapRange)
        tvMapRange = findViewById(R.id.tvMapRange)
        btnMapZoomIn = findViewById(R.id.btnMapZoomIn)
        btnMapZoomOut = findViewById(R.id.btnMapZoomOut)
        swWaypoints = findViewById(R.id.swWaypoints)
        btnWpAdd = findViewById(R.id.btnWpAdd)
        btnWpManage = findViewById(R.id.btnWpManage)
        swHistory = findViewById(R.id.swHistory)
        btnHistoryManage = findViewById(R.id.btnHistoryManage)
        drawer = findViewById(R.id.drawer)
        rowMeasure = findViewById(R.id.rowMeasure)
        btnAddPoint = findViewById(R.id.btnAddPoint)
        btnBreakChain = findViewById(R.id.btnBreakChain)
        btnUndoPoint = findViewById(R.id.btnUndoPoint)
        btnResetPoints = findViewById(R.id.btnResetPoints)

        // 两个开关的**记忆**。在挂监听**之前**恢复 —— 否则恢复这个动作本身会把监听跑一遍
        // （`swRecord` 那条会去 `startRecording()`，虽然那时 SLAM 没跑、会被自己挡掉）。
        swRecord.isChecked = pref.getBoolean(PREF_RECORD, false)
        // 测距也默认关。它和录制一样是**用户拨开才生效**的功能，不替用户决定。
        swMeasure.isChecked = pref.getBoolean(PREF_MEASURE, false)
        // ⚠️ 顺手推给 C++ —— 那边默认是关的，不在这儿推一次，两边会不一致。
        SlamBridge.nativeSetMeasuring(swMeasure.isChecked)

        // 相机线程和相机对象活得和 Activity 一样久（和录制页同一个理由：
        // start/stop 只是一条队列上的两个消息，顺序天然确定）。
        val ct = HandlerThread("camera").also { it.start() }
        camThread = ct
        cam = CameraCapture(this, Handler(ct.looper))
        cam.onFrame = ::onCameraFrame
        cam.logCapabilities()

        btnSlam.setOnClickListener { onButtonSlam() }
        findViewById<Button>(R.id.btnMenu).setOnClickListener {
            drawer.openDrawer(GravityCompat.END)
        }
        // 开关是**用户的意图**，录制跟着它走：启动 SLAM 时若开着就自动录，
        // 运行中拨动也立刻生效。所以它一直可点（见 render）。
        swRecord.setOnCheckedChangeListener { _, checked ->
            pref.edit().putBoolean(PREF_RECORD, checked).apply()
            if (checked) startRecording() else stopRecording()
        }

        // 测距同理，只是它作用在画面上而不是磁盘上。**关掉时清掉叠加层** ——
        // 不然后面 ticker 不再刷它，那个十字会一直挂在画面上。
        swMeasure.setOnCheckedChangeListener { _, checked ->
            pref.edit().putBoolean(PREF_MEASURE, checked).apply()
            SlamBridge.nativeSetMeasuring(checked)
            if (!checked) measureState = PreviewView.MeasureState.NO_TRACK
            // 「多点折线」那一行跟着它出现/消失。**已钉住的点留着** —— 拨开关不清会话，
            // 要清就按「重置」（或者等地图重置，见 pollMeasure）。
            render()
        }

        // 小地图：开关 + 切片厚度。同样在挂监听**之前**恢复。
        swMap.isChecked = pref.getBoolean(PREF_MAP, true)
        sbMapRange.max = kRangeSeekMax
        sbMapRange.progress = rangeToSeek(pref.getFloat(PREF_MAP_RANGE, MapView.SLICE_DEFAULT))
        applyMapRange(sbMapRange.progress)

        swMap.setOnCheckedChangeListener { _, checked ->
            pref.edit().putBoolean(PREF_MAP, checked).apply()
            applyMapVisible(checked)
        }
        // 先按当前开关同步一次：C++ 那边默认是开的，而用户可能记成了关。
        applyMapVisible(swMap.isChecked)

        sbMapRange.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            // ⚠️ 边拖边生效（`fromUser` 才落盘）—— 拖的时候要能当场看见切片变厚变薄。
            override fun onProgressChanged(sb: SeekBar?, progress: Int, fromUser: Boolean) {
                applyMapRange(progress)
                if (fromUser) pref.edit().putFloat(PREF_MAP_RANGE, seekToRange(progress)).apply()
            }

            override fun onStartTrackingTouch(sb: SeekBar?) {}
            override fun onStopTrackingTouch(sb: SeekBar?) {}
        })

        // 缩放档位要记着：上次按到哪一档，这次进来还是哪一档。
        // `-1` = 还没存过，`setZoomIndex` 会忽略它、用视图自己的默认档。
        mapView.setZoomIndex(pref.getInt(PREF_MAP_ZOOM, -1))
        zoomSaved = mapView.zoomIndex()
        // 只有加/减两个按钮；**长按连按**由 wireZoomButton 那一侧负责。
        wireZoomButton(btnMapZoomIn) {
            mapView.zoomIn()
            saveZoom()
        }
        wireZoomButton(btnMapZoomOut) {
            mapView.zoomOut()
            saveZoom()
        }

        // 多点折线：加点 / 断开 / 撤销 / 重置。**折线整个存在 C++ 侧**（它要存 `MapPoint*`），
        // 这一侧只调它、然后把结果读回来画。
        btnAddPoint.setOnClickListener { addPinnedPoint() }
        btnBreakChain.setOnClickListener { breakChain() }
        btnUndoPoint.setOnClickListener {
            SlamBridge.nativeUndoPinned()
            refreshPinned()
        }
        btnResetPoints.setOnClickListener {
            SlamBridge.nativeClearPinned()
            refreshPinned()
        }

        // 路径点。开关只影响**画不画**；管理界面始终能看（否则关掉开关就再也找不到它们了）。
        swWaypoints.isChecked = pref.getBoolean(PREF_WAYPOINTS, true)
        swWaypoints.setOnCheckedChangeListener { _, checked ->
            pref.edit().putBoolean(PREF_WAYPOINTS, checked).apply()
            // 关掉时 `clearWaypoints()`；打开时必须**强制重读**，否则 revision 没变会一直空着。
            wpRevisionSeen = -1
            refreshWaypoints()
        }
        btnWpAdd.setOnClickListener { askAddWaypoint() }
        btnWpManage.setOnClickListener { showWaypointManager() }

        // 历史轨迹：开关只管画不画；逐条隐藏 / 删除在管理页（开关关着也能管）。
        histHidden = (pref.getStringSet(PREF_HIST_HIDDEN, emptySet()) ?: emptySet())
            .toMutableSet()
        swHistory.isChecked = pref.getBoolean(PREF_HISTORY, true)
        // 这行在挂监听**之前**，不会触发 listener；而 `applyMapVisible` 已经先跑过一轮
        // （那时开关还是默认的关）⟹ 不置脏的话第一轮会停在那结果上，历史一直不出来。
        histDirty = true
        swHistory.setOnCheckedChangeListener { _, checked ->
            pref.edit().putBoolean(PREF_HISTORY, checked).apply()
            histDirty = true
            render()
        }
        btnHistoryManage.setOnClickListener { showHistoryManager() }

        // ⚠️ 叠加层**跟着画面走**，不跟着 ticker —— 否则标记会拖着画面跑（见 overlaySupplier）。
        preview.overlaySupplier = { if (swMeasure.isChecked) buildOverlay() else null }

        mapFile = intent.getStringExtra(EXTRA_MAP_FILE)
        Log.i(TAG, "地图来源：" + (mapFile ?: "新建地图"))
        // 载入地图的话，会话目录就是 `.osa` 的父目录。**提前**定下来（原先在 startHardware
        // 里才定）—— 这样没启动 SLAM 也能看 / 管理这张图的历史轨迹。
        mapFile?.let { f ->
            File(f).parentFile?.let { dir ->
                dir.mkdirs()
                sessionDir = dir
            }
        }

        // 读一次构造状态：可能是**上一个 Activity 实例**留下的（从入口页再进来一次）。
        // System 是进程级的，Kotlin 这边每次都要重新问，不能自己记。
        //
        // ⚠️ 这一句还是**第一次碰 `SlamBridge`** —— `.so` 在此刻加载，`JNI_OnLoad`
        // 顺带装上日志重定向（ORB-SLAM3 的 cout/cerr 全靠它）。别把它排到后面去。
        slamState = SlamBridge.nativeState()

        // `System` 是进程级的：已经造好的话，入口页这次的选择必须**手动生效** ——
        // `nativeInit` 不会再跑一遍。不换的话「新建地图」只是又开一张当前图，
        // 旧 atlas 原样留着（回环检测照样能匹配到它）；「换一张图」同理。
        var atlasSwitch = false
        if (slamState == SLAM_READY || slamState == SLAM_LOADING) {
            val want = mapFile?.removeSuffix(".osa") ?: ""
            val have = SlamBridge.nativeLoadedMap()
            if (want != have) {
                Log.i(TAG, "atlas 换源：$have → $want")
                atlasSwitching = true
                SlamBridge.nativeRequestLoadAtlas(want)
                atlasSwitch = true
            }
        }

        if (slamState == SLAM_LOADING || atlasSwitch) uiQueueTicker()

        render()
    }

    override fun onStop() {
        super.onStop()
        // 切后台等同「停止」（§7.3）。回来要重新按「启动」—— 后台相机会被限帧，
        // 而且手机可能移动了，上一帧的位姿不再成立。
        stopHardware("切后台")
    }

    override fun onDestroy() {
        super.onDestroy()
        stopHardware("退出")
        cam.onFrame = null
        uiHandler.removeCallbacksAndMessages(null)
        camThread.quitSafely()
    }

    // ---------------- 「准备 / 启动 / 结束 SLAM」 ----------------
    //
    // 和录制页那个两段式对齐：
    //
    //     未准备 ──[准备 SLAM]──► 加载中（词袋 38 s）──► 就绪
    //                                                    │
    //                                          [启动 SLAM] ▼
    //                                                  运行中 ──[结束]──► 就绪
    //
    // ⚠️ **没有「取消准备」** —— 和录制页不一样。`System` 的构造没法中途取消，
    // 而且造好之后**也没有安全的销毁时机**（它没有析构函数、`Shutdown()` 又不 join 线程，
    // delete 就是 use-after-free）。所以这个按钮是**单行道**：准备过就回不去「未准备」。
    // 这不是偷懒，是把 ORB-SLAM3 的事实如实反映到界面上。

    private fun onButtonSlam() {
        // —— 运行中：结束（结束会话，那张图成为「旧图」，§2.3）——
        if (slamRunning) {
            stopHardware("用户结束")
            render()
            return
        }
        // 加载中 / 失败：不响应
        if (slamState == SLAM_LOADING || slamState == SLAM_FAILED) return

        if (!hasCameraPermission()) {
            ActivityCompat.requestPermissions(
                this, arrayOf(Manifest.permission.CAMERA), REQ_CAMERA)
            return
        }

        // —— System 已经造好了：只开硬件，不重等那 38 秒（§5 决策一）——
        if (slamState == SLAM_READY) {
            openHardware()
            render()
            return
        }

        // —— 第一次：构造 System ——
        val vocab = resolveVocabPath() ?: return
        val settings = resolveSettingsPath() ?: return
        // 选了图就把路径一路传下去。C++ 那边还不支持载入旧图（会打一行「收到但没实现」），
        // 但通道先通着，免得等到 M8 才发现送不过去。
        SlamBridge.nativeInit(vocab, settings, mapFile)
        slamState = SlamBridge.nativeState()   // 正常是 kLoading
        render()
        uiQueueTicker()
    }

    /**
     * 开相机 + IMU + `nativeStart()`。
     *
     * **等 `System` 就绪之后才调** —— 词袋加载那 38 秒里灌帧没人消费，队列满了纯属白丢。
     */
    private fun openHardware() {
        if (slamRunning || !hasCameraPermission()) return

        // IMU 先起步（本机从注册到出数要 3 秒多，正好和相机启动重叠）。
        val t = HandlerThread("imu").also { it.start() }
        val c = ImuCollector(this)
        imuThread = t
        imu = c
        c.onSample = { ts, wx, wy, wz, ax, ay, az ->
            // **跑在 IMU 线程上，里面绝不能阻塞**（铁律 1）。
            try {
                SlamBridge.nativePushImu(ts, wx, wy, wz, ax, ay, az)
            } catch (e: Throwable) {
                Log.e(TAG, "喂 IMU 失败", e)
            }
        }
        c.start(Handler(t.looper))

        // 相机帧硬按住 300 ms，让 IMU 先写满 —— 录制模式一直有这道闸，M5 漏了。
        // 少了它 IMU 可能比相机晚开始，第一帧就没有预积分。
        framesFromTsNs = SystemClock.elapsedRealtimeNanos() + PREROLL_NS

        preview.setActive(true)
        cam.start()
        cam.beginSettle()   // 收敛对焦然后锁死 —— 锁焦的后果是 fx 只在那个距离上等于 435.2

        SlamBridge.nativeStart()   // 顺带把 C++ 那边的轨迹和重置计数清零
        slamRunning = true

        // 会话目录只建一次；本趟的轨迹和录像都放在它下面。
        //
        // 载入了旧图的话，会话目录就是那张图所在的目录（不新建）：这样一趟趟的轨迹
        // 会攒在同一张图下面，「同一张图的轨迹」不用另外存一份。
        if (sessionDir == null) {
            val loaded = mapFile?.let { File(it).parentFile }
            if (loaded != null) {
                loaded.mkdirs()
                sessionDir = loaded   // 不写 session.json —— 那是这张图第一次建的时候写的
            } else {
                newSessionDir().also {
                    sessionDir = it
                    writeSessionMeta(it)
                }
            }
        }
        // 一趟固定一个时间戳；轨迹先落 `traj/`，存图成功才归档到 `map-<id>/`。
        runStamp = stamp()
        curTrajBase = null
        curRecDir = null
        runMapId = -1
        runMerges = emptyList()
        histDirty = true   // 新一趟：显示用的图 id / 合并链都会变
        if (swRecord.isChecked) startRecording()

        Log.i(TAG, "SLAM 开始吃帧")
        // ⚠️ 必须自己把 ticker 叫起来：它在词袋加载完的那一刻就停了（busy 变 false），
        // 而位姿是靠它刷的。不叫的话界面会停在「已就绪」一动不动。
        uiQueueTicker()
    }

    /**
     * 关硬件。**幂等**，「停止」/ 切后台 / 退出共用。
     *
     * ⚠️ **不销毁 `System`** —— 那要重付 38 秒。C++ 那边 `slam::stop()` 只是停止吃帧，
     * 顺便清空积压的队列（否则下次启动会吃到停止前的旧帧）。
     */
    private fun stopHardware(reason: String) {
        if (!slamRunning) return
        Log.i(TAG, "SLAM 停止吃帧（$reason）")

        SlamBridge.nativeStop()   // 先让它别吃了，再拆硬件

        // ⚠️ 紧跟 `nativeStop()` 读：快照就在 `slam::stop()` 里做的。
        // 再晚一步，worker 可能已经消费「恢复 atlas」请求、把整张图换掉了。
        runMapId = SlamBridge.nativeLastTrajMapId()
        runMerges = readMerges()
        slamRunning = false

        stopRecording()           // 正在录的话文件一律保留（录到一半也是真数据）

        // 轨迹先落 `traj/`；存图成功那一刻才归档进 `map-<id>/`（见 applyPendingSave）。
        // **主动结束**才弹窗问留不留；切后台 / 退出直接留着（数据是真的）。
        val base = newTrajBase()
        if (base != null) {
            curTrajBase = base
            exportTrajectory(base)
            // 元信息**当场就写**，不等「留不留」那个弹窗 —— 存图可能在用户回答之前完成，
            // 那时归档会把这两个文件一起搬走；晚写就会一个在 map/、一个留在 traj/。
            writeRunMeta(base)
            // 先问地图、再问轨迹（askMapSave 问完会接着问轨迹）。一次只弹一个。
            if (reason == "用户结束") askMapSave(base) else keepRun(base)
        }

        cam.stop()
        imu?.stop()
        imuThread?.quitSafely()
        // 必须置 null：不置的话下次读到的是上一轮那个已经停掉的 collector。
        imu = null
        imuThread = null
        // ⚠️ 只清画面。叠加层不用管 —— 它现在**跟着新帧走**（`overlaySupplier`），
        // 没帧就没有叠加层，也就不用担心「没有画面的十字挂在黑底上」。
        preview.setActive(false)
        measureState = PreviewView.MeasureState.NO_TRACK
        // ticker 停了，小窗不会再被喂 ⟹ 在这儿补一次，**把整趟的图定格下来**（复盘好看）。
        // 历史要先刷：hint 取决于这一拍有没有东西可画。
        refreshHistory()
        refreshMap()
    }

    // ---------------- 帧出口 ----------------

    /**
     * 每帧的出口。**跑在相机线程上**，实现里不能有阻塞操作。
     *
     * **三个消费者，各自一道显式闸、各自包 try** —— 一路出岔子不能把另外两路吃掉。
     */
    private fun onCameraFrame(gray: ByteArray, tsNs: Long) {
        // 预热期内的帧不喂也不录；预览不受影响。
        val warm = tsNs >= framesFromTsNs

        // ① 喂 SLAM。一条 JNI 调用 + 一次拷贝就返回。
        if (slamRunning && warm) {
            try {
                SlamBridge.nativePushImage(tsNs, gray, CameraCapture.W, CameraCapture.H)
            } catch (e: Throwable) {
                Log.e(TAG, "喂帧失败", e)
            }
        }

        // ② 落盘（抽屉里的开关）。
        val w = writer
        if (recording && warm && w != null) {
            try {
                w.offerFrame(SessionWriter.FrameItem(tsNs, gray, CameraCapture.W, CameraCapture.H))
            } catch (e: Throwable) {
                Log.e(TAG, "落盘丢帧", e)
            }
        }

        // ③ 预览。
        try {
            preview.offer(gray, tsNs)
        } catch (e: Throwable) {
            Log.e(TAG, "预览丢帧", e)
        }
    }

    // ---------------- 落盘开关（抽屉里）----------------

    private fun startRecording() {
        if (!slamRunning) {
            Log.w(TAG, "SLAM 没在跑，不能录")
            return
        }
        if (writer != null) return
        val session = sessionDir
        if (session == null || runStamp.isEmpty()) {
            Log.w(TAG, "没有会话目录，不能录")
            return
        }

        // 一趟一个 `rec/<runStamp>/`；中途开录制也落在同一个底下。
        val run = curRecDir
            ?: File(TrajectoryStore.recDir(session), runStamp).also { curRecDir = it }

        // ⚠️ **每一段写自己的目录**（`session-1/`、`session-2/`…）。不能复用同一个 ——
        // `SessionWriter.start()` 用 `FileOutputStream` 打开 csv 会**截断**，
        // 而上一段的 PNG 还在磁盘上，结果 csv 和 PNG 对不上，而且坏得很隐蔽。
        val dir = nextSessionDir(run)
        val w = SessionWriter(dir)
        // ⚠️ 标出来：这样录的和「录制模式」录的才分得清 —— 格式一模一样，
        // 而这段的帧率是被落盘拖低过的，用途也不同（见 SessionWriter.withSlam）。
        w.withSlam = true
        w.start()
        if (w.ioError != null) {
            Log.e(TAG, "建 session 文件失败：${w.ioError}")
            tvStatus.text = "无法创建输出文件：${w.ioError}\n$dir"
            swRecord.isChecked = false   // 写不进去就别让开关骗人
            return
        }
        writer = w
        recording = true
        imu?.attach(w)
        Log.i(TAG, "开始录制（带 SLAM）$dir")
        render()
    }

    private fun stopRecording() {
        if (!recording) return
        recording = false
        imu?.detach()
        val w = writer
        writer = null
        w?.stop()
        if (w != null) {
            Log.i(TAG, "停止录制  相机 ${w.framesWritten.get()} 帧 / IMU ${w.imuWritten.get()} 条")
        }
        render()
    }

    // ---------------- 位姿与轨迹 ----------------

    /** [base] 不含扩展名 —— 轨迹写 `base.txt`，元信息写 `base.json`。 */
    private fun exportTrajectory(base: File) {
        try {
            SlamBridge.nativeDumpTrajectory("$base.txt")
        } catch (e: Throwable) {
            Log.e(TAG, "导出轨迹失败", e)
        }
    }

    // ---------------- 状态栏 ----------------

    /**
     * **加载中要跑，运行中也要跑。**
     *
     * 两个都必需：加载中是为了刷那 38 秒的进度；运行中是为了刷位姿和跟踪状态。
     * ⚠️ 而「刚变成 `kReady` 的那一刻」两件事同时发生 —— 开硬件也是在那一刻做的，
     * 所以这里不能一看到不是 `kLoading` 就 return。
     */
    private val ticker = object : Runnable {
        override fun run() {
            if (slamState == SLAM_LOADING) {
                slamState = SlamBridge.nativeState()
                slamElapsedMs = SlamBridge.nativeElapsedMs()
                // ⚠️ **不要在这儿开硬件。** 「就绪」和「启动」是两个阶段，
                // 中间隔着用户那一下 —— 自动开会把三段式黏成两段（改标签时漏改过一版）。
            }
            if (slamRunning) {
                poseValid = SlamBridge.nativeGetPose(poseBuf)
                trkState = SlamBridge.nativeGetTrackingState()
                pollMeasure()
            }

            // ⚠️ **先刷，再判断要不要继续。** 反过来写的话，「词袋加载完」那一轮会
            // 读到 READY、算出 busy=false、然后在 render **之前**就 return ——
            // 界面永远停在「加载中…（已 40s）」，时间不走、按钮也不变。
            // **最后一轮刷新被自己吃掉了。**
            render()

            if (slamState == SLAM_LOADING || slamRunning || saving || restoring) {
                uiHandler.postDelayed(this, TICK_MS)
            }
        }
    }

    private fun uiQueueTicker() {
        uiHandler.removeCallbacks(ticker)
        uiHandler.post(ticker)
    }

    /**
     * 取一次画面中心的距离读数，交给 [PreviewView] 画。
     *
     * ⚠️ 「跟踪未就绪」要**单独一档**。C++ 只回 0/1/2（不可测/勉强/好），而跟踪没就绪时
     * 它回的也是 0 —— 直接显示成「对准有纹理的地方」是**错的话**，用户会去擦镜头。
     * 那个状态属于 SLAM，在这儿叠上去（STAGE6 §2.7：沉默和「测不了」必须能区分）。
     */
    private fun pollMeasure() {
        // ⚠️ **这儿不画东西**。叠加层由 `overlaySupplier` 跟着新帧拉（15fps），
        // 在这儿推的话只有 ticker 那 4Hz，标记会拖尾。这里只管**状态**。
        if (!swMeasure.isChecked) return

        // ⚠️ 地图一重置，世界坐标系**重新锚定** ⟹ 已钉住的点是废图里的坐标，画出来是乱飘的。
        // 用 `>` 而不是 `!=`：`nativeStart()` 会把计数归零，那不算重置。
        val resets = SlamBridge.nativeResetCount()
        if (resets > resetCountSeen) {
            SlamBridge.nativeClearPinned()
            segLast.clear()
            segLabels.clear()
        }
        resetCountSeen = resets

        readMeasure()
    }

    /**
     * 读一次画面中心的测量，顺手把 [measureState] 定下来。返回 C++ 那个质量码。
     *
     * ⚠️ **两处都调它**：ticker 上那次（4Hz）管**按钮的灰亮**，`buildOverlay` 里那次（15Hz）
     * 管**画**。少任何一处都会「一半新一半旧」。
     */
    private fun readMeasure(): Int {
        val q = SlamBridge.nativeGetMeasure(measureBuf)
        measureState = if (trkState != TRK_OK) PreviewView.MeasureState.NO_TRACK else when (q) {
            2 -> PreviewView.MeasureState.GOOD
            1 -> PreviewView.MeasureState.FAIR
            else -> PreviewView.MeasureState.NONE
        }
        return q
    }

    // ---------------- 多点折线 ----------------

    /**
     * 折线变了：读一次 + 刷按钮状态。
     *
     * **不用手动重画叠加层** —— 它跟着下一帧（≤66ms）自己就更新，见 [PreviewView.overlaySupplier]。
     */
    private fun refreshPinned() {
        readPinned()
        render()
    }

    /**
     * 读一次折线到 [pinnedBuf]，点数写进 [pinnedN]。**主线程**。
     *
     * ⚠️ 容量不够时 C++ **只写装得下的那几个**（它返回「实际写了几个」—— 这样调用方
     * 不会去读没写过的内存）。所以看到「写满」就扩容。
     *
     * **留 8 个点的余量**，而加点一次只加一个、每次加完立刻重读 ⟹ **正常用法碰不到截断**。
     * 真撞上了也只是**那一帧少画一个点**，下一帧就好（不为「批量加载」那种不存在的场景
     * 加一层重读循环）。
     */
    private fun readPinned(): Int {
        pinnedN = SlamBridge.nativeGetPinned(pinnedBuf)
        // ⚠️ **保持余量，不是「满了才扩」。** 满了才扩的话那一帧**已经被 C++ 截断了**
        // （它只写装得下的那几个），会少画几个点。余量掉到 [kPinnedSlack] 以下就提前扩，
        // 于是**截断那条路正常永远走不到**。
        // 和现有大小一起翻倍，是为了别每加一个点就重分配一次。
        val cap = (pinnedBuf.size - 1) / 8   // C++ 最多能写几个点（那个 -1 留给橡皮筋那块）
        if (cap - pinnedN < kPinnedSlack) {
            pinnedBuf = FloatArray(maxOf(8 * (pinnedN + kPinnedSlack) + 1, pinnedBuf.size * 2))
        }
        return pinnedN
    }

    /**
     * 段标签。**只在变化超过 0.5mm 时才重新 `format`。**
     *
     * ⚠️ 长度现在**每帧都在变** —— 地图一直在收敛，两点的距离跟着变（这是刷新要的效果，
     * 不是抖动）。所以不能像以前那样「只在点变了时算」；但也不该每帧都 `String.format`
     * （几十微秒 × 段数 × 15fps 是毫秒级）。显示到毫米，**比半毫米更小的变化不该让
     * 字符串重排** —— 那只会让标签闪。
     */
    private fun segLabel(i: Int, v: Float): String {
        while (segLast.size <= i) {
            segLast.add(-1f)
            segLabels.add("")
        }
        if (segLast[i] < 0f || abs(v - segLast[i]) > 5e-4f) {
            segLast[i] = v
            segLabels[i] = "%.3f m".format(v)
        }
        return segLabels[i]
    }

    private fun buildOverlay(): PreviewView.Overlay {
        // ⚠️ **这里也要读一次测量。** 叠加层是 15fps 画的，而 ticker 上的 `pollMeasure`
        // 只有 4Hz —— 只靠它的话，标记和折线是平滑的，**十字读数、琥珀吸附圈、橡皮筋的
        // 另一端却在一秒跳 4 次**，一头平滑一头在跳，比全都慢更难看。
        readMeasure()
        val n = readPinned()
        val marks = ArrayList<PreviewView.Mark>(n)
        val segs = ArrayList<PreviewView.Seg>(maxOf(0, n - 1))
        var live: PreviewView.Seg? = null

        // 每点 3 个：`px py flags`。**标记点**只画在相机前方的。
        for (i in 0 until n) {
            val o = 1 + i * 3
            val f = pinnedBuf[o + 2].toInt()
            if (f and kFlagFront == 0) continue
            marks.add(PreviewView.Mark(pinnedBuf[o], pinnedBuf[o + 1], f and kFlagLow != 0))
        }
        // 每段 5 个：`x0 y0 x1 y1 len` —— **两个端点都是 C++ 算好的**，
        // 包括「一端在相机后面」时那个朝正确方向的画外点。这里只认 `len`。
        for (i in 0 until n - 1) {
            val o = segAt(i)
            if (pinnedBuf[o + 4] < 0f) continue
            segs.add(PreviewView.Seg(pinnedBuf[o], pinnedBuf[o + 1], pinnedBuf[o + 2], pinnedBuf[o + 3],
                                     segLabel(i, pinnedBuf[o + 4])))
        }
        // 橡皮筋：最后一个点 → 十字。**断开 / 没读数 / 两端都在相机后面时那侧给的是 -1**。
        if (n > 0) {
            val lo = segAt(n - 1)
            if (pinnedBuf[lo + 4] >= 0f) {
                live = PreviewView.Seg(pinnedBuf[lo], pinnedBuf[lo + 1],
                                       pinnedBuf[lo + 2], pinnedBuf[lo + 3],
                                       "%.3f m".format(pinnedBuf[lo + 4]))
            }
        }
        return PreviewView.Overlay(measureState, measureBuf[0], measureBuf[4], measureBuf[5],
                                   marks, segs, live)
    }

    /**
     * 把当前读数钉成一个点。
     *
     * 先试**吸到已有点**：用**上一帧那份缓冲**里的投影判像素距离（≤66ms 旧，够用）——
     * 因为用户是按**屏幕上看到的**判断「是不是同一个角」的，世界距离阈值在不同景深下
     * 含义完全不同。
     *
     * ⚠️ 判出下标之后交给 C++ **复制那个点的身份**，不是「把当前读数钉下去」——
     * 后者存的是十字命中的**另一个**地图点（只是碰巧很近），之后两个点会**各漂各的**。
     */
    private fun addPinnedPoint() {
        if (!measureState.hasReading) return

        var snapTo = -1
        val cx = CameraCapture.W * 0.5f
        val cy = CameraCapture.H * 0.5f
        for (i in 0 until pinnedN) {
            val o = 1 + i * 3
            if (pinnedBuf[o + 2].toInt() and kFlagFront == 0) continue
            val dx = pinnedBuf[o] - cx
            val dy = pinnedBuf[o + 1] - cy
            if (dx * dx + dy * dy <= kPointSnapPx * kPointSnapPx) {
                snapTo = i
                break
            }
        }
        if (SlamBridge.nativeAddPinned(snapTo)) refreshPinned()
    }

    /**
     * 「断开」：当前这条折线到此为止。
     *
     * **点全部留着**，只是不再从最后一个点拉橡皮筋，下一个「加点」会**另起一条**。
     * 「开没开」完全从读回来的 flags 推（见 [chainOpen]），这一侧一条链状态都不存 ——
     * 那样「撤销」就得按情况还原它，两边很容易对不上。
     */
    private fun breakChain() {
        SlamBridge.nativeClosePinnedChain()
        refreshPinned()
    }

    private fun render() {
        // ---- atlas 恢复 ----
        // 只在这一侧轮询：C++ 那边在 SLAM 线程上重建 atlas，主线程问不得。
        val rs = SlamBridge.nativeRestoreState()
        restoring = rs == 1 || rs == 2
        if (rs != restoreStateSeen) {
            restoreStateSeen = rs
            restoreMsg = when (rs) {
                1, 2 -> "正在载入地图…"
                3 -> if (atlasSwitching) "地图已切换" else "已丢弃这一趟：地图退回上一次保存"
                4 -> "载入地图失败（看 logcat 的 slam）—— 内存里现在是一张空图"
                else -> restoreMsg
            }
        }

        // ---- 按钮 ----
        // 三个标签对应三个阶段。**加载中和构造失败都不可点** —— 前者没法取消，后者重试也没用
        // （C++ 那边的 `init` 只生效一次）。恢复期间也不可点：atlas 正在换。
        btnSlam.isEnabled =
            (slamRunning || slamState == SLAM_IDLE || slamState == SLAM_READY) && !restoring
        btnSlam.text = when {
            // 加载中 / 构造失败优先：换源请求这会儿还没轮到（失败的话永远不会轮到）。
            slamState == SLAM_LOADING -> "加载中…"
            slamState == SLAM_FAILED -> "构造失败"
            restoring -> "载入地图…"
            slamRunning -> "结束"
            slamState == SLAM_READY -> "启动 SLAM"
            else -> "准备 SLAM"
        }
        btnSlam.backgroundTintList =
            if (slamRunning) ColorStateList.valueOf(RED) else null
        btnSlam.setTextColor(if (slamRunning) Color.WHITE else DARK_TEXT)

        // 两个开关都**一直可点** —— 它们表达的是「这一段要不要录 / 要不要量」，
        // 所以启动之前就得能设。
        swRecord.isEnabled = true
        swMeasure.isEnabled = true

        // ---- 多点折线。**绑在测距开关上**：关着就整行消失 ----
        rowMeasure.visibility = if (swMeasure.isChecked) View.VISIBLE else View.GONE
        // 「加点」没有效读数时**灰着**。按了没反应，用户分不清是坏了还是没对准（STAGE6 §2.7）；
        // 灰着 + 十字旁边那句「对准有纹理的地方」已经说清为什么。
        btnAddPoint.isEnabled = swMeasure.isChecked && measureState.hasReading
        // 「断开」只在折线**还开着**的时候有意义（已经断开了再按是空操作 ⟹ 灰着）。
        btnBreakChain.isEnabled = swMeasure.isChecked && chainOpen
        btnUndoPoint.isEnabled = pinnedN > 0
        btnResetPoints.isEnabled = pinnedN > 0

        // 放路径点：跑着且有位姿才可点。
        btnWpAdd.isEnabled = slamRunning && poseValid
        btnWpManage.isEnabled = SlamBridge.nativeWaypointCount() > 0

        // 加路径点的结果：**只提示一次**（`wpState` 是粘性的，会一直是 2/3）。
        val wpState = SlamBridge.nativeWaypointState()
        if (wpState != wpStateSeen) {
            wpStateSeen = wpState
            when (wpState) {
                3 -> wpMsg = "放路径点失败：${SlamBridge.nativeWaypointMsg()}"
                2 -> wpMsg = null
            }
        }

        // ---- 状态栏 ----
        val mapLabel = mapFile?.let { File(it).name } ?: "新建地图"
        // `System` 造好之后有两种情形，先算出来 —— 塞进 `when` 的 else 分支里会写出
        // 「一个 when 里两个 else」那种读起来像错的东西。
        val afterReady = if (slamRunning) {
            trackingLine()
        } else {
            "SLAM：已就绪（没在吃帧）\n\n点「启动 SLAM」开始 —— 相机和 IMU 这一刻才打开。"
        }
        val head = when (slamState) {
            SLAM_IDLE ->
                "SLAM：未准备（$mapLabel）\n\n点「准备 SLAM」构造 System，约 38 秒。"
            SLAM_LOADING -> "SLAM：词袋加载中…（已 ${slamElapsedMs / 1000}s，本机约 38 秒）"
            SLAM_FAILED ->
                if (SlamBridge.nativeFailureCode() == 1)
                    "SLAM：载图失败 —— 这张图读不了，回入口页换一张或新建"
                else
                    "SLAM：构造失败 —— 看 logcat（tag slam）"
            else -> afterReady
        }

        // 存图状态转成人话。文件名不用问 C++ —— 就是这一侧传过去的。
        when (SlamBridge.nativeSaveState()) {
            1, 2 -> saveMsg = "正在保存地图…"
            3 -> {
                // ⚠️ 只在「保存中 → 完成」这一拍归档。`saveState` 是**粘性的**（一直是 3），
                // 之后任何一次 render 都会再看到它 —— 不加这个闸，坐标会被变换很多遍。
                val justFinished = saving
                saveMsg = "地图已保存 → ${savedName ?: "?"}.osa"
                saving = false
                if (justFinished) {
                    applyPendingSave()
                    // 文件夹内容变了（归档 / 合并重组）。脏标记放调用点：
                    // applyPendingSave 有好几条提前 return 的路径。
                    histDirty = true
                }
            }
            4 -> {
                saveMsg = "地图保存失败（看 logcat 的 slam）"
                saving = false
                pendingArchive = null   // 磁盘不动，松文件留在 traj/
            }
        }

        val body = buildString {
            if (slamRunning && poseValid) {
                append(
                    "位置 m: %.3f, %.3f, %.3f".format(poseBuf[12], poseBuf[13], poseBuf[14])
                )
            }
            if (slamRunning) {
                // ⚠️ 重置次数是**初始化问题最直接的证据** —— 归零重来多少次，一眼看到。
                if (isNotEmpty()) append("\n")
                append("轨迹 ${SlamBridge.nativeTrajSize()} 点")
                val resets = SlamBridge.nativeResetCount()
                if (resets > 0) append("　⚠️ 地图已重置 $resets 次")
                // ⚠️ 只加**纹理**，不加距离 —— 这一行太长会**折行**，`tvStatus` 一变高
                // 就把预览挤下去（那个距离十字旁边本来就有）。
                // **世界坐标也不在这儿显**：它是给折线和日志用的，这行放不下
                // （要看那三个数读 logcat，tag `slam` 的 `[M6]`）。
                if (swMeasure.isChecked) {
                    // 纹理 = 十字附近有几个**特征点**（不管有没有地图点）。
                    // **没读数时也有值** ⟹ 正好告诉用户「这地方为什么测不了」。
                    append("　纹理 %.0f".format(measureBuf[6]))
                }
            }
            val w = writer
            if (recording && w != null) {
                append("\n录制中：相机 ${w.framesWritten.get()} 帧 / IMU ${w.imuWritten.get()} 条")
            }
            // 地图账。载图之后这是界面上唯一看得出「图进来了」的地方 ——
            // 载进来的图不是当前图，小窗、轨迹、测距都看不到它。
            // ⚠️ 恢复期间不读：`Atlas::PostLoad()` 在重建整个 atlas，它没拿锁。
            if (slamState == SLAM_READY && !restoring) {
                SlamBridge.nativeAtlasStats(atlasBuf)
                if (isNotEmpty()) append("\n")
                append("地图 ${atlasBuf[0]} 张 / 关键帧 ${atlasBuf[1]} / 点云 ${atlasBuf[2]}")
                // 路径点：有才显示，免得平时多占一行。
                val wpTotal = SlamBridge.nativeWaypointCount()
                if (wpTotal > 0) append("　路径点 $wpTotal")
            }
            // 词袋加载期间那条「正在载入地图…」只是排队，别和加载进度叠在一起。
            if (slamState != SLAM_LOADING) restoreMsg?.let { append("\n$it") }
            saveMsg?.let { append("\n$it") }
            wpMsg?.let { append("\n$it") }
            lastRunDir?.let { append("\n上一趟：${it.absolutePath}") }
        }
        tvStatus.text = if (body.isEmpty()) head else "$head\n\n$body"

        // 2D 地图小窗也由这儿喂 —— 于是 ticker、按钮事件、以及 idle 的首帧全覆盖。
        // 恢复期间跳过：轨迹表刚被清、点云快照也刚被清，读出来是一片空。
        // **历史先刷**：`refreshMap` 的 hint 要看这一拍有没有历史可画。
        if (!restoring) {
            refreshHistory()
            refreshMap()
            refreshWaypoints()
        }
    }

    // ---------------- 路径点 ----------------

    /**
     * 读一次路径点喂给小地图。
     *
     * ⚠️ **不能按 revision 跳过**：回环/合并会把整张图的位姿改掉（`ApplyScaledRotation`），
     * 坐标变了而 revision 不变，只认 revision 的话标记会一直停在旧位置（看着就是「没了」）。
     * 点只有几个，每拍重读的代价可以忽略。
     *
     * 只把**有效且在当前图**的交给 [MapView]：别的图在另一个坐标系里，画上去必然错位。
     */
    private fun refreshWaypoints() {
        if (!swWaypoints.isChecked) {
            mapView.clearWaypoints()
            return
        }

        // 和 `readPinned` 一个契约：装不下就少给，看到写满就扩容重来。
        while (true) {
            val n = SlamBridge.nativeGetWaypoints(wpPos, wpMeta, wpNames)
            if (n < wpPos.size / 3 && n < wpMeta.size / 4 && n < wpNames.size) {
                wpN = n
                break
            }
            wpPos = FloatArray(wpPos.size * 2)
            wpMeta = LongArray(wpMeta.size * 2)
            wpNames = arrayOfNulls(wpNames.size * 2)
        }

        // 挑出「有效 + 在当前图」的，压成 `MapView` 要的那种扁平数组。
        val xs = FloatArray(wpN * 2)
        val ns = arrayOfNulls<String>(wpN)
        var k = 0
        for (i in 0 until wpN) {
            val flags = wpMeta[i * 4 + 3]
            if ((flags and kFlagWpValid) == 0L || (flags and kFlagWpCurrent) == 0L) continue
            xs[k * 2] = wpPos[i * 3]
            xs[k * 2 + 1] = wpPos[i * 3 + 1]
            ns[k] = wpNames[i]
            k++
        }
        mapView.setWaypoints(xs, k, ns)
    }

    /** 加路径点：先问名字，再发请求。**名字可以当场改**。 */
    private fun askAddWaypoint() {
        val input = EditText(this).apply {
            // 用**下一个 id** 而不是「现有条数 + 1」：删过点之后条数会回退，
            // 那样新点会和还没删的那个重名。
            setText("路径点 ${SlamBridge.nativeNextWaypointId()}")
            setSelectAllOnFocus(true)
            setPadding(56, 0, 56, 0)
        }
        AlertDialog.Builder(this)
            .setTitle("添加路径点")
            .setView(input)
            .setPositiveButton("添加") { _, _ ->
                val name = input.text.toString().trim().ifEmpty { "路径点" }
                Log.i(TAG, "加路径点「$name」")
                SlamBridge.nativeRequestAddWaypoint(name)
                wpStateSeen = 1
                uiQueueTicker()
            }
            .setNegativeButton("取消", null)
            .show()
    }

    /** 管理界面：每条路径点一行，点了就能改名或删掉。**含不在当前图 / 已失效的**。 */
    private fun showWaypointManager() {
        val total = SlamBridge.nativeWaypointCount()
        if (total == 0) {
            AlertDialog.Builder(this)
                .setTitle("路径点")
                .setMessage("还没有路径点。\n\n在 SLAM 跑起来之后，走到想记的地方点「添加路径点」。")
                .setPositiveButton("好", null)
                .show()
            return
        }

        val pos = FloatArray(3 * total)
        val meta = LongArray(4 * total)
        val names = arrayOfNulls<String>(total)
        val n = SlamBridge.nativeGetWaypoints(pos, meta, names)

        val labels = Array(n) { i ->
            val name = names[i] ?: "路径点"
            val flags = meta[i * 4 + 3]
            val mapId = meta[i * 4 + 1]
            when {
                (flags and kFlagWpValid) == 0L -> "$name · 已失效"
                (flags and kFlagWpCurrent) == 0L -> "$name · 图 #$mapId（不在当前图）"
                else -> "$name · 图 #$mapId"
            }
        }

        AlertDialog.Builder(this)
            .setTitle("路径点（$n）")
            .setItems(labels) { _, which ->
                val id = meta[which * 4].toInt()
                val name = names[which] ?: "路径点"
                AlertDialog.Builder(this)
                    .setTitle(name)
                    .setItems(arrayOf("重命名", "删除")) { _, pick ->
                        when (pick) {
                            0 -> askRenameWaypoint(id, name)
                            else -> {
                                SlamBridge.nativeDeleteWaypoint(id)
                                refreshWaypoints()
                                render()
                            }
                        }
                    }
                    .show()
            }
            .setNegativeButton("关闭", null)
            .show()
    }

    private fun askRenameWaypoint(id: Int, old: String) {
        val input = EditText(this).apply {
            setText(old)
            setSelectAllOnFocus(true)
            setPadding(56, 0, 56, 0)
        }
        AlertDialog.Builder(this)
            .setTitle("重命名")
            .setView(input)
            .setPositiveButton("确定") { _, _ ->
                val name = input.text.toString().trim().ifEmpty { old }
                SlamBridge.nativeRenameWaypoint(id, name)
                refreshWaypoints()
                render()
            }
            .setNegativeButton("取消", null)
            .show()
    }

    // ---------------- 2D 地图小窗 ----------------

    /**
     * 读一次轨迹和点云塞给小窗。**纯读 + 换引用**，算和画都在 [MapView] 那边。
     *
     * 小地图关着时**整个不读** —— 这两条读的都是要拿锁的活：
     * 轨迹走四条 `std::list` 抱 `mMutexTraj`（tracking 每帧都要拿），
     * 点云是 SLAM 线程那份快照（每秒重建一次，重建时抱地图的锁）。
     * 藏起来时白付不起，见 [applyMapVisible]。
     */
    private fun refreshMap() {
        if (mapWrap.visibility != View.VISIBLE) return

        val n = SlamBridge.nativeGetTrajectory(trajBuf)
        val pn = SlamBridge.nativeGetMapPoints(mapPtBuf)
        // ⚠️ **不沉默**：空窗也要说清为什么空。**分开四态**（STAGE6 §2.7）——
        // 「加载中」和「没启动」混成一句的话，用户会去擦镜头。
        // ⚠️ 判据是「**两样都没有**」：有点云 / 轨迹 / 历史可画时再拿提示盖上去，
        //    那些就白出来了。停止之后也保留最后一趟 + 历史轨迹（复盘用）。
        val anything = n > 0 || pn > 0 || histShown > 0
        val hint = when {
            slamState == SLAM_LOADING -> "词袋加载中…"
            slamState == SLAM_FAILED -> "构造失败"
            !anything && !slamRunning -> "SLAM 未启动"
            !anything && trkState != TRK_OK -> "跟踪未就绪"
            !anything -> "等待定位…"
            else -> null
        }
        // 停止后不再画当前位置：`poseValid` 是最后一帧的旧值，历史轨迹的坐标系
        // 可能已经换过（恢复 atlas）。视图中心由 MapView 冻住，不会跳。
        val hasCur = slamRunning && poseValid
        // 朝向：相机光轴投到 XY 地面。`poseBuf` 是 **Twc**（列主序），
        // 所以**第 3 列**就是相机系 `+z`（光轴）在世界里的方向。
        // ⚠️ 只挡**除零**（真的正下方，方位角无定义）。**不挡「朝下」** ——
        //    低头测地板时那点水平分量仍然是**稳定的方位角**（晃 1° 就动 1°）。
        var dx = 0f
        var dy = 0f
        if (hasCur) {
            val fx = poseBuf[8]
            val fy = poseBuf[9]
            val len = Math.hypot(fx.toDouble(), fy.toDouble()).toFloat()
            if (len > 1e-6f) {
                dx = fx / len
                dy = fy / len
            }
        }
        mapView.submit(MapView.Snapshot(
            trajBuf, n,
            if (hasCur) poseBuf[12] else 0f,
            if (hasCur) poseBuf[13] else 0f,
            if (hasCur) poseBuf[14] else 0f,
            hasCur, hint, dx, dy,
            mapPtBuf, pn))
    }

    // ---------------- 历史轨迹 ----------------

    /**
     * 读一次「当前这张图」的历史轨迹喂给小地图。
     *
     * 只画轨迹所属那张图的文件夹（`map-<id>/`）—— 别的图各有一个原点，叠上来必然错位。
     * 合并还没落盘时文件夹要乘 `T_on`；`nativeDisplayTransforms` 已经按链合成好了。
     *
     * ⚠️ 只在上下文变了 / 置脏时列目录、读文件。ticker 是 4 Hz，每拍读盘白付不起。
     */
    private fun refreshHistory() {
        if (restoring) return

        val nTf = SlamBridge.nativeDisplayTransforms(histTfBuf)
        val ctx = buildString {
            append(sessionDir?.absolutePath).append('|').append(nTf)
            for (i in 0 until nTf * 9) append(',').append(histTfBuf[i])
        }
        if (ctx == histCtx && !histDirty) return
        histCtx = ctx
        histDirty = false

        val session = sessionDir
        histEntries = if (nTf == 0 || session == null) emptyList() else {
            val list = ArrayList<TraceEntry>()
            for (i in 0 until nTf) {
                val mapId = histTfBuf[i * 9].toLong()
                val tf = histTfBuf.copyOfRange(i * 9, i * 9 + 9)
                for (f in TrajectoryStore.tracesFor(session, mapId)) {
                    list += TraceEntry(f, mapId, tf)
                }
            }
            list
        }
        histAll = histEntries.size
        btnHistoryManage.isEnabled = histAll > 0
        if (histAll > 0) Log.i(TAG, "历史轨迹：图 #${histTfBuf[0].toLong()} · $histAll 条")

        if (!swHistory.isChecked || mapWrap.visibility != View.VISIBLE) {
            histShown = 0
            mapView.clearHistory()
            return
        }

        val traces = ArrayList<FloatArray>()
        for (e in histEntries) {
            if (e.file.absolutePath in histHidden) continue
            val n = SlamBridge.nativeReadTrajFile(
                e.file.absolutePath,
                e.tf[1], e.tf[2], e.tf[3], e.tf[4], e.tf[5], e.tf[6], e.tf[7], e.tf[8],
                histPtsBuf)
            if (n > 0) traces += histPtsBuf.copyOf(n * 3)
        }
        // 单点的轨迹画不出来（`MapView` 要 ≥ 2 点），别让它把 hint 顶掉。
        histShown = traces.count { it.size >= 6 }
        mapView.setHistory(traces)
    }

    /** 管理当前图的已归档轨迹：逐条显示 / 隐藏、删除。 */
    private fun showHistoryManager() {
        // 重新算一次条目：用户可能刚在别处删过 / 加过。
        histDirty = true
        refreshHistory()
        val entries = histEntries
        if (entries.isEmpty()) {
            AlertDialog.Builder(this)
                .setTitle("历史轨迹")
                .setMessage(
                    "这张图还没有已保存的轨迹。\n\n" +
                        "轨迹在存图成功之后才会从 traj/ 归档进 map-<id>/。"
                )
                .setPositiveButton("好", null)
                .show()
            return
        }

        val labels = Array(entries.size) { i ->
            val e = entries[i]
            val stamp = e.file.name.removePrefix("traj-").removeSuffix(".txt")
            val pts = readTracePoints(e.file)
            buildString {
                append(stamp).append(" · 图 #").append(e.mapId)
                if (pts >= 0) append(" · ").append(pts).append(" 点")
                if (e.file.absolutePath in histHidden) append(" · 已隐藏")
            }
        }
        AlertDialog.Builder(this)
            .setTitle("历史轨迹（${entries.size}）")
            .setItems(labels) { _, which -> showTraceActions(entries[which]) }
            .setNegativeButton("关闭", null)
            .show()
    }

    private fun showTraceActions(e: TraceEntry) {
        val hidden = e.file.absolutePath in histHidden
        AlertDialog.Builder(this)
            .setTitle(e.file.name)
            .setItems(arrayOf(if (hidden) "显示" else "隐藏", "删除")) { _, pick ->
                if (pick == 0) setTraceHidden(e.file, !hidden) else askDeleteTrace(e.file)
            }
            .setNegativeButton("取消", null)
            .show()
    }

    private fun setTraceHidden(f: File, hidden: Boolean) {
        if (hidden) histHidden.add(f.absolutePath) else histHidden.remove(f.absolutePath)
        // 不能改 `getStringSet` 返回的那个对象；存一份新的。
        pref.edit().putStringSet(PREF_HIST_HIDDEN, histHidden.toSet()).apply()
        histDirty = true
        render()
    }

    private fun askDeleteTrace(f: File) {
        AlertDialog.Builder(this)
            .setTitle("删除这条轨迹？")
            .setMessage(
                "${f.name}\n\n" +
                    "只删轨迹数据（.txt）和元信息（.json）；已保存的地图（.osa）不受影响。"
            )
            .setPositiveButton("删除") { _, _ ->
                f.delete()
                File(f.absolutePath.removeSuffix(".txt") + ".json").delete()
                histDirty = true
                render()
            }
            .setNegativeButton("取消", null)
            .show()
    }

    /** 轨迹的元信息里那个点数；没有 / 读不到返回 -1。 */
    private fun readTracePoints(txt: File): Int = try {
        val meta = File(txt.absolutePath.removeSuffix(".txt") + ".json")
        if (!meta.isFile) -1
        else Regex("\"traj_points\"\\s*:\\s*(\\d+)").find(meta.readText())
            ?.groupValues?.get(1)?.toInt() ?: -1
    } catch (e: Exception) {
        -1
    }

    // ---------------- 小地图的开关和切片厚度 ----------------

    /**
     * 显示 / 隐藏小地图。
     *
     * **关掉时连 C++ 那边也停**：点云快照那次枚举要抱 `Map::mMutexMap`（几毫秒，
     * 红黑树指针追逐），而 LocalMapping 每建一个点都要拿它。藏起来时白付不起。
     * 重开时下一个帧间空隙就重建，不会空一下。
     *
     * 轨迹的**记录**停不了也不该停（那是 ORB-SLAM3 自己在 push，`trajectory.txt` 靠它），
     * 但那个 4 Hz 的**读取**跟着一起停了 —— 见 [refreshMap]。
     */
    private fun applyMapVisible(on: Boolean) {
        mapWrap.visibility = if (on) View.VISIBLE else View.GONE
        // 别让人去调一个看不见的东西。
        sbMapRange.isEnabled = on
        sbMapRange.alpha = if (on) 1f else 0.4f
        tvMapRange.alpha = if (on) 1f else 0.4f
        SlamBridge.nativeSetMapCloudEnabled(on)
        // 藏起来时 `refreshHistory` 会清掉历史层；重新打开要重读一遍。
        histDirty = true
        refreshHistory()
        refreshMap()
    }

    /** 滑块刻度 → 切片厚度（**米，全宽**）。 */
    private fun seekToRange(p: Int): Float =
        MapView.SLICE_MIN + (MapView.SLICE_MAX - MapView.SLICE_MIN) * p / kRangeSeekMax

    private fun rangeToSeek(m: Float): Int =
        ((m - MapView.SLICE_MIN) / (MapView.SLICE_MAX - MapView.SLICE_MIN) * kRangeSeekMax)
            .roundToInt().coerceIn(0, kRangeSeekMax)

    private fun applyMapRange(progress: Int) {
        val m = seekToRange(progress)
        mapView.setSliceRange(m)
        tvMapRange.text = "地图高度范围　%.1f m".format(m)
    }

    // ---------------- 缩放键 ----------------

    /**
     * 缩放键：**点一下走一档，按住不放连续走**。松手（或滑出去取消）就停。
     *
     * 档位是 ×1.5 递下去的、有十几档，一路点过去太费手。
     */
    private fun wireZoomButton(b: Button, zoom: () -> Unit) {
        b.setOnClickListener { zoom() }
        b.setOnLongClickListener {
            stopZoomRepeat()
            val r = object : Runnable {
                override fun run() {
                    zoom()
                    uiHandler.postDelayed(this, kZoomRepeatMs)
                }
            }
            zoomRepeat = r
            uiHandler.post(r)   // 长按判定一成立就当场走一档，不用再等一拍
            true                // 消费掉 ⟹ 长按之后不会再补一次点击
        }
        // ⚠️ 返回 **false**：别把事件吃掉，点击和长按照常走 View 自己那套。
        b.setOnTouchListener { _, e ->
            if (e.action == MotionEvent.ACTION_UP || e.action == MotionEvent.ACTION_CANCEL) {
                stopZoomRepeat()
            }
            false
        }
    }

    private fun stopZoomRepeat() {
        zoomRepeat?.let { uiHandler.removeCallbacks(it) }
        zoomRepeat = null
    }

    /** 缩放档位落盘。**只在真的换档时写** —— 长按连按时这个函数每秒会被叫十几次。 */
    private fun saveZoom() {
        val i = mapView.zoomIndex()
        if (i == zoomSaved) return
        zoomSaved = i
        pref.edit().putInt(PREF_MAP_ZOOM, i).apply()
    }

    /**
     * 跟踪状态那一行。
     *
     * `NOT_INITIALIZED` 那一条要把话说明白：**惯性初始化需要运动激励，静止举着手机
     * 永远初始化不了** —— 用户会以为坏了（STAGE7 §2.3 的引导文案）。
     */
    private fun trackingLine(): String = when (trkState) {
        TRK_OK -> "● 跟踪中（OK）"
        TRK_RECENTLY_LOST -> "… 跟踪不稳，放慢动作"
        TRK_LOST -> "… 已丢失，对着刚才扫过的地方"
        TRK_NOT_INITIALIZED -> "… 惯性初始化中 —— 请缓慢走动，静止举着不会初始化"
        TRK_NO_IMAGES -> "… 还没收到画面"
        else -> "… 还没准备好（跟踪状态 $trkState）"
    }

    // ---------------- 权限 ----------------

    private fun hasCameraPermission() =
        ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) ==
            PackageManager.PERMISSION_GRANTED

    override fun onRequestPermissionsResult(
        requestCode: Int, permissions: Array<out String>, grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode != REQ_CAMERA) return
        if (grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            // 把刚才那一下就接着做完。
            onButtonSlam()
        } else {
            tvStatus.text = "相机权限被拒绝。\n请到 设置 → 应用 → SlamCapture → 权限 里手动开启。"
        }
    }

    // ---------------- 文件 ----------------

    /**
     * 词袋路径。145 MB，没进 APK，靠 `tools/push-vocab.sh` 推到外部文件目录。
     *
     * 返回 null = 已经往状态栏写了可读的提示，调用方直接返回、**不要调 nativeInit**。
     */
    private fun resolveVocabPath(): String? {
        val f = File(getExternalFilesDir(null) ?: filesDir, VOCAB_NAME)
        return f.takeIf { it.isFile && it.length() > 0 && it.canRead() }?.absolutePath
            ?: refuse("词袋", f) { "跑一次 tools/push-vocab.sh 把它推上来（145 MB，不进 APK）。" }
    }

    /**
     * yaml 路径。`assets/phone.yaml` 是权威副本，但 C++ 那边要的是**真实文件路径**
     * （`cv::FileStorage` 不吃 AssetManager），所以先落到私有目录。
     */
    private fun resolveSettingsPath(): String? {
        val dst = File(filesDir, SETTINGS_NAME)
        try {
            assets.open(SETTINGS_NAME).use { input ->
                dst.outputStream().use { output -> input.copyTo(output) }
            }
            // 选了旧图就把路径补进去。`System` 构造时读的就是这个键，所以载图只在
            // 那一刻生效一次。值不带 `.osa`，上游自己加后缀。
            mapFile?.let { dst.appendText("\nSystem.LoadAtlasFromFile: \"${it.removeSuffix(".osa")}\"\n") }
        } catch (e: IOException) {
            Log.e(TAG, "释放 $SETTINGS_NAME 失败", e)
            tvStatus.text = "打不开 assets/$SETTINGS_NAME：$e"
            return null
        }
        return dst.takeIf { it.length() > 0 && it.canRead() }?.absolutePath
            ?: refuse("配置", dst) { "assets/$SETTINGS_NAME 是空的，检查打包。" }
    }

    /**
     * **第一道防线**（STAGE5 §3.2）。路径不对时 ORB-SLAM3 用 `exit(-1)` 报错，
     * 表现是「App 整个退回桌面、logcat 干干净净」—— 能在这里挡住的就别让它跑进去。
     *
     * 挡不住的也得知道它挡不住：yaml **存在但缺一个必填键**照样会 exit(-1)，
     * 那种只能靠 M2 接进 logcat 的 `cerr`。
     */
    private fun refuse(what: String, f: File, hint: () -> String): String? {
        Log.e(TAG, "找不到$what：${f.absolutePath}")
        tvStatus.text = "找不到$what：\n${f.absolutePath}\n\n${hint()}"
        return null
    }

    /**
     * 一次**会话**（一次「新建地图 / 打开地图」）= 一个目录：`files/maps/<yyyyMMdd-HHmmss>/`。
     *
     * **用时间戳不用 `mapId`** —— mapId 会随进程重启和新建图变，时间戳稳定；
     * 而且「另存为新图」天然就是新目录。
     */
    private fun newSessionDir(): File {
        // getExternalFilesDir 可能返回 null（外部存储没挂载）。这时退到内部存储，
        // **绝不能**用 File(null, ...)——那会落到进程的 CWD，在 Android 上就是 "/"。
        val maps = File(getExternalFilesDir(null) ?: filesDir, "maps")
        // 时间戳只到秒，同一秒里连开两次会撞名 —— 而撞了就会有两个 .osa 落进同一目录。
        val s = stamp()
        var d = File(maps, s)
        var n = 2
        while (d.exists()) {
            d = File(maps, "$s-$n")
            n++
        }
        return d.apply { mkdirs() }
    }

    /** 目录名和地图文件名的统一时间戳格式。 */
    private fun stamp(): String = SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(Date())

    /** `<rec/<runStamp>>/session-M/`，M 取第一个没被占的号。理由见 [startRecording]。 */
    private fun nextSessionDir(run: File): File {
        var n = 1
        while (File(run, "session-$n").exists()) n++
        return File(run, "session-$n")
    }

    /** 这一趟已经录出来的段（`rec/<runStamp>/session-M/`）。 */
    private fun sessionDirs(run: File?): List<File> =
        run?.listFiles { f -> f.isDirectory && f.name.startsWith("session-") }
            ?.sorted() ?: emptyList()

    /**
     * 本趟轨迹的前缀：`<会话>/traj/traj-<runStamp>`（不含扩展名）。
     *
     * 落在这里而不是 `map-<id>/` —— 图还没存下来之前，那个 id 随时可能被回滚复用。
     */
    private fun newTrajBase(): File? {
        val session = sessionDir ?: return null
        if (runStamp.isEmpty()) return null
        val dir = TrajectoryStore.trajDir(session)
        if (!dir.exists() && !dir.mkdirs()) {
            Log.e(TAG, "建轨迹目录失败：${dir.absolutePath}")
            return null
        }
        return File(dir, "traj-$runStamp")
    }

    /** 读本趟的合并链（`slam::stop()` 里快照好的）。每条 10 个 float。 */
    private fun readMerges(): List<FloatArray> {
        val buf = FloatArray(10 * 16)
        val n = SlamBridge.nativeLastMerges(buf)
        return (0 until n).map { i -> buf.copyOfRange(i * 10, i * 10 + 10) }
    }

    // ---------------- 这一趟留不留 ----------------

    /**
     * 弹窗问「这一趟留不留」。
     * ⚠️ **「不保存」必须是明确动作** —— 点外面 / 按返回都算保存，免得手滑丢数据。
     */
    private fun askKeepRun(base: File) {
        val segs = sessionDirs(curRecDir).size
        val resets = SlamBridge.nativeResetCount()
        val msg = buildString {
            append("轨迹 ${SlamBridge.nativeTrajSize()} 点")
            if (runMapId >= 0) append(" · 图 #$runMapId")
            if (segs > 0) append("，会话 $segs 段")
            append("\n")
            if (runMerges.isNotEmpty())
                append("本趟合并：" + runMerges.joinToString("，") {
                    "图 #${it[0].toLong()} ← 图 #${it[1].toLong()}"
                } + "\n")
            if (resets > 0) append("地图被重置 $resets 次 —— 轨迹只保留了最后一段\n")
            append("\n${base.parentFile?.absolutePath ?: base.absolutePath}")
        }
        AlertDialog.Builder(this)
            .setTitle("保存这一趟？")
            .setMessage(msg)
            .setPositiveButton("保存") { _, _ -> keepRun(base) }
            .setNegativeButton("不保存") { _, _ -> onDropTrajectory(base) }
            .setOnCancelListener { keepRun(base) }
            .show()
    }

    /**
     * 结束时的**第一问**：地图存不存。问完（不管选了哪个）接着问轨迹。
     *
     * 排在轨迹前面是有原因的：存地图要**当场把文件名定下来**，定完之后写出来的
     * 轨迹 json 才记得上它。
     */
    private fun askMapSave(trajBase: File) {
        SlamBridge.nativeAtlasStats(atlasBuf)
        val dir = sessionDir
        if (dir == null || atlasBuf[1] <= 0) {
            // 这一趟没建出图 —— 没什么好问的，但也别沉默。
            saveMsg = "这一趟没有可存的地图"
            askKeepRun(trajBase)
            return
        }

        val existing = dir.listFiles { f -> f.isFile && f.name.endsWith(".osa") }
            ?.maxByOrNull { it.lastModified() }

        // 只有「另存为」用得上。不勾就只拷轨迹（几毫秒），
        // 勾了连录像一起拷 —— 那是原始数据，一段可能几百 MB。
        val copyRec = CheckBox(this).apply {
            text = "另存时连录像一起拷"
            setPadding(56, 0, 56, 0)
        }

        val b = AlertDialog.Builder(this)
            .setTitle("保存地图？")
            .setMessage(
                buildString {
                    append("关键帧 ${atlasBuf[1]} / 点云 ${atlasBuf[2]}\n\n${dir.absolutePath}")
                    // 这个 .osa 装的是整个 atlas。载入过旧图、本趟又建出了新图就是两张
                    //（Atlas::PreSave 只会剔掉没有关键帧的空图）。
                    if (atlasBuf[0] > 1)
                        append("\n\n注意：这个 .osa 里会有 ${atlasBuf[0]} 张图，不只是当前这一张。")
                }
            )
            .setView(copyRec)
        // 用户到底选没选「存」。三个肯定按钮都要置 true ——
        // 只有「不保存」和直接关掉（返回键 / 点外面）才该丢图。
        var willSave = false
        if (existing != null) {
            b.setPositiveButton("覆盖") { _, _ ->
                willSave = true
                startMapSave(existing.absolutePath.removeSuffix(".osa"))
            }
                .setNeutralButton("另存为") { _, _ ->
                    willSave = true
                    saveMapAs(copyRec.isChecked)
                }
                .setNegativeButton("不保存", null)
        } else {
            b.setPositiveButton("保存") { _, _ ->
                willSave = true
                startMapSave(File(dir, stamp()).absolutePath)
            }
                .setNegativeButton("不保存", null)
        }
        val dlg = b.create()
        // 挂在「弹窗消失」上：两个轴互不影响，问完接着问轨迹。
        dlg.setOnDismissListener {
            if (!willSave) {
                // 这一趟不要了：把 atlas 退回上一次保存的样子。
                // 不退回的话，这趟建的图会一直留在内存里，下一趟的回环检测还能匹配到它。
                Log.i(TAG, "这一趟不存图 → 请求恢复 atlas")
                atlasSwitching = false
                SlamBridge.nativeRequestRestoreMap()
                uiQueueTicker()   // 恢复的进度和结果要靠 ticker 刷
            }
            askKeepRun(trajBase)
        }
        dlg.show()
    }

    /** 让 SLAM 线程把地图存成 `path + ".osa"`（`path` 不含扩展名）。 */
    private fun startMapSave(pathNoExt: String) {
        savedName = File(pathNoExt).name
        saveMsg = "正在保存地图…"
        Log.i(TAG, "存图 → $pathNoExt.osa")
        // 先存归档上下文：存图完成时这一趟可能已经被下一趟顶掉了。
        sessionDir?.let { pendingArchive = PendingArchive(it, runMapId, runMerges, curTrajBase) }
        SlamBridge.nativeRequestSaveMap(pathNoExt)
        // 刚停止，ticker 本来要停了 —— 叫起来才刷得到「已保存」。
        saving = true
        uiQueueTicker()
        render()
    }

    /**
     * 另存：新开一个文件夹，把已经归档的图目录、还没归档的松轨迹拷过去，
     * 当前这趟也算在新图里。录像只在勾了的时候带（见 [askMapSave]）。
     */
    private fun saveMapAs(copyRecordings: Boolean) {
        val src = sessionDir ?: return
        val dst = newSessionDir()
        File(src, "session.json").takeIf { it.isFile }
            ?.copyTo(File(dst, "session.json"), overwrite = true)

        src.listFiles { f -> f.isDirectory && f.name.startsWith("map-") }
            ?.forEach { copyTree(it, File(dst, it.name)) }
        TrajectoryStore.trajDir(src).takeIf { it.isDirectory }
            ?.let { copyTree(it, TrajectoryStore.trajDir(dst)) }
        if (copyRecordings) {
            TrajectoryStore.recDir(src).takeIf { it.isDirectory }
                ?.let { copyTree(it, TrajectoryStore.recDir(dst)) }
        }

        // 本趟的松文件也换位置了 —— **指针必须跟着改**，否则存图成功后会去旧目录搬。
        curTrajBase?.let { base ->
            curTrajBase = File(TrajectoryStore.trajDir(dst), base.name)
        }

        sessionDir = dst   // 之后这一趟的东西都算这张新图的
        Log.i(TAG, "另存为新图：${dst.absolutePath}")
        startMapSave(File(dst, dst.name).absolutePath)
    }

    private fun copyTree(src: File, dst: File) {
        dst.mkdirs()
        src.listFiles()?.forEach { f ->
            if (f.isDirectory) copyTree(f, File(dst, f.name))
            else f.copyTo(File(dst, f.name), overwrite = true)
        }
    }

    private fun keepRun(base: File) {
        lastRunDir = base.parentFile
        Log.i(TAG, "保留这一趟：${base.absolutePath}")
        render()
    }

    /**
     * 用户不要这一趟的轨迹了。
     *
     * ⚠️ **录过会话就再问一次** —— 那段得走一趟才录得到，成本比轨迹高得多，
     * 不该跟着轨迹一起没。没录过就直接删。
     */
    private fun onDropTrajectory(base: File) {
        val segs = sessionDirs(curRecDir).size
        if (segs == 0) {
            discardAll(base)
            return
        }
        AlertDialog.Builder(this)
            .setTitle("这一趟录了 $segs 段会话")
            .setMessage(
                "要保留吗？\n\n" +
                    "「保留会话」只删掉本趟的轨迹和它的 json；" +
                    "「全部删除」连录像一起删。\n\n${base.absolutePath}"
            )
            .setPositiveButton("保留会话") { _, _ -> dropTrajectoryOnly(base) }
            .setNegativeButton("全部删除") { _, _ -> discardAll(base) }
            .setOnCancelListener { dropTrajectoryOnly(base) }
            .show()
    }

    /**
     * 本趟轨迹现在在哪 —— 可能还在 `traj/`，也可能已经被存图成功归档进 `map-<id>/`。
     *
     * ⚠️ 存图可能在「留不留轨迹」那个弹窗回答之前就完成 ⟹ 不查这里的话，
     * 「不保存」会删一个不存在的路径，文件反而留在图目录里。
     */
    private fun runTrajFile(base: File, ext: String): File {
        val direct = File("${base}$ext")
        if (direct.isFile) return direct
        val session = sessionDir
        if (session != null && runMapId >= 0) {
            val archived = File(TrajectoryStore.mapDir(session, runMapId), base.name + ext)
            if (archived.isFile) return archived
        }
        return direct
    }

    /** 只删本趟的轨迹和 json，留下录像。 */
    private fun dropTrajectoryOnly(base: File) {
        runTrajFile(base, ".txt").delete()
        runTrajFile(base, ".json").delete()
        lastRunDir = curRecDir
        Log.i(TAG, "丢掉轨迹、保留会话：${base.absolutePath}")
        render()
    }

    /**
     * 整趟不要了：轨迹 + 录像都删。
     *
     * ⚠️ **不能删会话目录** —— `map-<id>/` 里是别的趟的轨迹，`rec/` 里也可能是别的趟的录像。
     */
    private fun discardAll(base: File) {
        Log.i(TAG, "丢弃这一趟：${base.absolutePath}")
        runTrajFile(base, ".txt").delete()
        runTrajFile(base, ".json").delete()
        curRecDir?.deleteRecursively()
        curRecDir = null
        lastRunDir = null
        render()
    }

    /**
     * `traj-<stamp>.json`：这一趟的元信息。
     * 轨迹只含跟踪 OK 的帧；地图每次重置后轨迹清空重来，所以只有最后一段。
     *
     * `merges` 记的是**坐标系怎么来的**：重组只在存图成功那一刻做一次，之后被吸收的
     * 目录就删了，链从磁盘上再也看不出来。
     */
    private fun writeRunMeta(base: File) {
        try {
            File("${base}.json").writeText(
                """
                {
                  "created": "${nowStamp()}",
                  "map_source": "${jsonEsc(mapSourceLabel())}",
                  "sessions": ${sessionDirs(curRecDir).size},
                  "traj_points": ${SlamBridge.nativeTrajSize()},
                  "map_resets": ${SlamBridge.nativeResetCount()},
                  "map_id": $runMapId,
                  "merges": ${mergesJson()}
                }
                """.trimIndent()
            )
        } catch (e: Exception) {
            Log.e(TAG, "写轨迹 json 失败", e)
        }
    }

    /** 合并链的 JSON：`[[survivor, absorbed, s, qx, qy, qz, qw, tx, ty, tz], …]`。 */
    private fun mergesJson(): String =
        runMerges.joinToString(", ", "[", "]") { r ->
            "[" + r[0].toLong() + ", " + r[1].toLong() + ", " +
                (2..9).joinToString(", ") { i -> r[i].toString() } + "]"
        }

    /**
     * 存图成功之后把本趟轨迹归档进 `map-<id>/`，并按合并链把磁盘上的坐标系统一。
     *
     * ⚠️ **只在「保存中 → 完成」那一拍调一次**（见 [render]）—— `saveState` 是粘性的。
     * ⚠️ 按链**逐条**做：每一步「幸存目录里现有的东西」正好在当时的坐标系里 ——
     *    先变换、再把被吸收目录原样搬进来。两条合并记录不能拆成「先全变换再全搬」。
     */
    private fun applyPendingSave() {
        val p = pendingArchive ?: return
        pendingArchive = null
        val session = p.session
        val id = p.mapId

        for (r in p.merges) {
            val survivor = r[0].toLong()
            val absorbed = r[1].toLong()
            val dir = TrajectoryStore.mapDir(session, survivor)

            // ① 存量轨迹按 `T_on` 变换。先全部写 `.tmp`，都成功了再统一换名。
            if (dir.isDirectory) {
                val files = dir.listFiles { f ->
                    f.isFile && f.name.startsWith("traj-") && f.name.endsWith(".txt")
                }.orEmpty()
                for (f in files) {
                    if (!SlamBridge.nativeTransformTrajFile(
                            f.absolutePath, "${f.absolutePath}.tmp",
                            r[2], r[3], r[4], r[5], r[6], r[7], r[8], r[9]
                        )
                    ) {
                        Log.e(TAG, "轨迹变换失败，重排中止：${f.absolutePath}")
                        dir.listFiles { g -> g.name.endsWith(".txt.tmp") }?.forEach { it.delete() }
                        return
                    }
                }
                for (g in dir.listFiles { f -> f.name.endsWith(".txt.tmp") }.orEmpty()) {
                    val target = File(g.absolutePath.removeSuffix(".tmp"))
                    if (!g.renameTo(target)) {
                        Log.e(TAG, "轨迹换名失败，重排中止：${g.absolutePath}")
                        return
                    }
                }
            }

            // ② 被吸收目录：**原样**搬进来（它本来就在最终坐标系里），空了才删。
            val src = TrajectoryStore.mapDir(session, absorbed)
            if (!src.isDirectory) continue
            val dst = dir.apply { mkdirs() }
            for (f in src.listFiles().orEmpty()) {
                if (!moveInto(f, dst)) {
                    Log.e(TAG, "合并目录中止：${src.absolutePath}")
                    return
                }
            }
            if (src.listFiles().isNullOrEmpty()) src.delete()
        }

        // ③ 本趟的松轨迹：已经在最终坐标系里，直接搬，**不参与任何变换**。
        if (id < 0) return   // 没有图：松文件留在 traj/
        val base = p.base ?: return
        val dst = TrajectoryStore.mapDir(session, id).apply { mkdirs() }
        moveInto(File("${base}.txt"), dst)
        moveInto(File("${base}.json"), dst)
        lastRunDir = dst
    }

    /** 把 [f] 搬进 [dst]；重名时加 `-2`。失败返回 false（调用方**不能**因此删源目录）。 */
    private fun moveInto(f: File, dst: File): Boolean {
        if (!f.isFile) return true
        val dot = f.name.lastIndexOf('.')
        val stem = if (dot > 0) f.name.substring(0, dot) else f.name
        val ext = if (dot > 0) f.name.substring(dot) else ""
        var target = File(dst, f.name)
        var n = 2
        while (target.exists()) {
            target = File(dst, "$stem-$n$ext")
            n++
        }
        val ok = f.renameTo(target)
        if (!ok) Log.e(TAG, "搬文件失败：${f.absolutePath} → ${target.absolutePath}")
        return ok
    }

    /**
     * `<会话>/session.json`：整个会话的汇总。建目录时写一次，之后不动。
     *
     * ⚠️ 名字故意不叫 `traj-*.json` —— 免得和趟级那份混淆（一个是会话级、一个是趟级）。
     */
    private fun writeSessionMeta(session: File) {
        try {
            File(session, "session.json").writeText(
                """
                {
                  "created": "${nowStamp()}",
                  "map_source": "${jsonEsc(mapSourceLabel())}",
                  "note": "一次「新建地图/打开地图」= 这个目录。轨迹按图分在 map-<id>/，未归档的在 traj/，录像在 rec/。"
                }
                """.trimIndent()
            )
        } catch (e: Exception) {
            Log.e(TAG, "写 session.json 失败", e)
        }
    }

    private fun mapSourceLabel(): String = mapFile?.let { File(it).name } ?: "新建地图"

    private fun jsonEsc(s: String): String = s.replace("\\", "\\\\").replace("\"", "\\\"")

    private fun nowStamp(): String =
        SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss", Locale.US).format(Date())
}
