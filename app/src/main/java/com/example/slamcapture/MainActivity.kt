package com.example.slamcapture

import android.Manifest
import android.content.pm.PackageManager
import android.content.res.ColorStateList
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.HandlerThread
import android.os.Looper
import android.os.SystemClock
import android.util.Log
import android.view.MotionEvent
import android.view.View
import android.widget.Button
import android.widget.FrameLayout
import android.widget.ImageView
import android.widget.LinearLayout
import android.widget.TextView
import androidx.activity.OnBackPressedCallback
import androidx.appcompat.app.AppCompatActivity
import androidx.appcompat.widget.SwitchCompat
import androidx.core.app.ActivityCompat
import androidx.core.content.ContextCompat
import androidx.core.view.GravityCompat
import androidx.drawerlayout.widget.DrawerLayout
import com.google.android.material.checkbox.MaterialCheckBox
import com.google.android.material.color.MaterialColors
import com.google.android.material.dialog.MaterialAlertDialogBuilder
import com.google.android.material.slider.Slider
import com.google.android.material.R as MaterialR
import java.io.File
import java.io.IOException
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import kotlin.math.abs
import kotlin.math.roundToInt

/**
 * SLAM 主界面。
 *
 * 相机和 IMU 由本页持有，与 [CaptureActivity] 各开各的。
 *
 * 主按钮有三个阶段：
 *   未准备 -> [准备 SLAM] -> 加载中（词袋约 38 秒）-> 就绪
 *   就绪 -> [启动 SLAM] -> 运行中 -> [结束] -> 就绪
 *
 * 准备阶段构造 System；启动阶段打开相机和 IMU 并开始吃帧。
 * 没有取消准备：System 构造无法取消，也没有安全的销毁时机。
 */
class MainActivity : AppCompatActivity() {

    companion object {
        private const val TAG = "MainActivity"

        /** 入口页选中的已保存地图绝对路径；没有该 extra 表示新建地图。 */
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

        /** 预热时长：延迟相机帧，让 IMU 先写入一段。 */
        private const val PREROLL_NS = 300_000_000L

        /** 「启动」后等对焦收敛的上限；到点就用兜底档位开跑。 */
        private const val FOCUS_WAIT_MS = 3_000L

        /** 没有屈光度读数时按 1 m 处理（内参表最常用的一档）。 */
        private const val DEFAULT_DIOPTER = 1.0f

        /** profile 里没量过 IMU 频率时的兜底（本机实测 402 Hz）。 */
        private const val IMU_FREQ_FALLBACK = 402f

        /** 开关状态的持久化。 */
        private const val PREF = "slamapp"
        private const val PREF_RECORD = "record"
        private const val PREF_MEASURE = "measure"
        private const val PREF_MAP = "map"
        private const val PREF_MAP_RANGE = "mapRange"
        /** 小地图缩放档位下标。 */
        private const val PREF_MAP_ZOOM = "mapZoom"
        private const val PREF_WAYPOINTS = "waypoints"
        /** 小地图是否绘制本趟轨迹。 */
        private const val PREF_MAP_TRAJ = "mapTraj"
        private const val PREF_HISTORY = "historyOn"
        /** 隐藏的历史轨迹绝对路径集合。 */
        private const val PREF_HIST_HIDDEN = "historyHidden"
        /** 查看器切片中心模式：false 跟随当前高度，true 使用点云 z 中位数。 */
        private const val PREF_VIEWER_SLICE = "viewerSliceMedian"

        /** 每条历史轨迹的最大绘制点数。 */
        private const val kHistoryMaxPts = 2000

        /** nativeDisplayTransforms 的缓冲容量。 */
        private const val kDisplayTfMax = 32

        /** 点云上限，与 slam_engine.cpp 的 kMapPointsMax 对应；使用固定缓冲。 */
        private const val kMapPointsMax = 5000

        /** 按住缩放键时的连发间隔。 */
        private const val kZoomRepeatMs = 95L

        /** 切片厚度滑块的步长，必须与布局里的 android:stepSize 一致。 */
        private const val kRangeStep = 0.05f

        /** 加点时吸附到已有点的半径（图像像素）。 */
        private const val kPointSnapPx = 20f

        /** 折线缓冲保留的最小点数余量。 */
        private const val kPinnedSlack = 8

        /** nativeGetPinned 每点 flags 的位，与 slam_engine.cpp 对应。 */
        private const val kFlagFront = 1   // 在相机前方
        private const val kFlagClosed = 2  // 它之后「断开」了
        private const val kFlagLow = 4     // 不太准，绘制空心圈

        /** 路径点 flags 的位，与 System::GetWaypoints 对应。 */
        private const val kFlagWpValid = 1L
        private const val kFlagWpCurrent = 2L
    }

    private lateinit var btnSlam: Button
    private lateinit var swRecord: SwitchCompat
    private lateinit var swMeasure: SwitchCompat
    private lateinit var tvStatus: TextView
    private lateinit var preview: PreviewView
    private lateinit var drawer: DrawerLayout

    // ---------------- 状态区 ----------------

    private lateinit var chipTrack: TextView
    private lateinit var chipPoints: TextView
    private lateinit var chipRecord: TextView
    private lateinit var chipMap: TextView
    private lateinit var bannerWrap: LinearLayout
    private lateinit var progress: com.google.android.material.progressindicator.LinearProgressIndicator
    private lateinit var tvDiag: TextView

    /** 主按钮的默认配色，离开「运行中」时还原。 */
    private var btnSlamTint: ColorStateList? = null
    private var btnSlamTextColor: Int = 0

    /** 一次性错误提示；下一次操作时清除。 */
    private var uiMessage: String? = null

    /** 短暂停留的成功提示，过期后自动收起。 */
    private var transientMsg: String? = null
    private var transientUntil = 0L
    private val transientClear = Runnable {
        if (SystemClock.elapsedRealtime() >= transientUntil) {
            transientMsg = null
            render()
        }
    }

    /** 跟踪提示的粘滞窗口：OK 与「不稳」交替时不至于每 250 ms 闪一次。 */
    private var trkMsg: String? = null
    private var trkMsgUntil = 0L

    private val uiHandler = Handler(Looper.getMainLooper())

    /** 开关状态的持久化。 */
    private val pref by lazy { getSharedPreferences(PREF, MODE_PRIVATE) }

    // ---------------- 页面状态 ----------------

    /** 入口页选中的已保存地图；null 表示新建地图。 */
    private var mapFile: String? = null

    /** C++ 侧 System 状态。只在主线程写。 */
    private var slamState = SLAM_IDLE
    private var slamElapsedMs = 0L

    /**
     * 是否正在吃帧：相机和 IMU 已打开，且已调用 nativeStart()。
     * 与 [slamState] 不同，System 就绪后仍会一直存在。
     */
    private var slamRunning = false

    // ---------------- 内参与对焦 ----------------

    /** 当前这一趟使用的内参；null 表示还没确定。 */
    private var activeRow: CalibRow? = null

    /** 内参来源的一句话，诊断面板显示。 */
    private var intrinsicsNote = "未确定"

    /** 正在等对焦收敛：相机已开，nativeStart() 还没调。 */
    private var waitingFocus = false

    /** 等对焦的截止时刻（elapsedRealtime）。 */
    private var focusDeadlineMs = 0L

    /** 当前相机 id 与传感器方向：写 yaml 和记录内参都要。 */
    private var cameraId = ""
    private var sensorOrientation = 0

    // ---------------- 硬件（本页自己持有）----------------

    private lateinit var camThread: HandlerThread
    private lateinit var cam: CameraCapture
    private var imu: ImuCollector? = null
    private var imuThread: HandlerThread? = null

    /** 预热截止时刻；早于该时刻的帧不喂给 SLAM，也不落盘。 */
    @Volatile
    private var framesFromTsNs = 0L

    // ---------------- 落盘（抽屉里的开关）----------------

    /** 落盘目标，只在录制时非 null；相机线程和 IMU 线程读取。 */
    @Volatile
    private var writer: SessionWriter? = null
    private var recording = false

    /**
     * 本次会话目录：files/maps/<时间戳>/。
     * 一次新建地图只创建一个目录，之后一直复用。
     */
    private var sessionDir: File? = null

    /** atlas 统计：[图数, 关键帧数, 地图点数]，复用数组。 */
    private val atlasBuf = IntArray(3)

    /** 最近一次存图的文件名（不含 .osa），成功后显示。 */
    private var savedName: String? = null

    /** 存图状态文字，在 render 中根据 nativeSaveState 更新。 */
    private var saveMsg: String? = null

    /** 是否正在存图。ticker 依赖它继续运行。 */
    private var saving = false

    /**
     * 是否正在恢复 atlas。
     * 恢复期间不读取 atlas 和轨迹；ticker 依赖它继续运行。
     */
    private var restoring = false

    /** 上次看到的恢复状态；只在变化时更新 [restoreMsg]。 */
    private var restoreStateSeen = 0

    /** 恢复状态文字。 */
    private var restoreMsg: String? = null

    /** 最近一次请求是否是入口页换源；仅影响提示语。 */
    private var atlasSwitching = false

    /** 本趟时间戳，轨迹和录像共用。 */
    private var runStamp = ""

    /** 本趟录像目录 <会话>/rec/<runStamp>/；未录制时为 null。 */
    private var curRecDir: File? = null

    /**
     * 本趟轨迹前缀（含目录、不含扩展名）：<会话>/traj/traj-<runStamp>。
     * 先写入 traj/，存图成功后才移动到 map-<id>/。
     */
    private var curTrajBase: File? = null

    /** 本趟轨迹所属的图，在 nativeStop() 后立即快照；-1 表示没有。 */
    private var runMapId = -1L

    /** 本趟合并链，每条 10 个 float。 */
    private var runMerges: List<FloatArray> = emptyList()

    /** 上次保留的目录，仅用于状态栏显示。 */
    private var lastRunDir: File? = null

    /**
     * 存图请求发出时的归档上下文。
     * 存图异步完成时可能已开始下一趟，归档必须使用请求时的快照。
     */
    private class PendingArchive(
        val session: File,
        val mapId: Long,
        val merges: List<FloatArray>,
        val base: File?
    )
    private var pendingArchive: PendingArchive? = null

    // ---------------- 位姿 ----------------

    /** 复用的位姿数组，避免每帧分配。 */
    private val poseBuf = FloatArray(16)
    private var poseValid = false
    private var trkState = TRK_NOT_READY

    // ---------------- 测距（画面中心）----------------

    /** 测量缓冲：[0] 距离、[1..3] 世界坐标、[4][5] 图像像素。 */
    private val measureBuf = FloatArray(7)

    /** 叠加层状态，比 C++ 的三档多一个「跟踪未就绪」。 */
    private var measureState = PreviewView.MeasureState.NO_TRACK

    // ---------------- 多点折线（绑在 swMeasure 那个开关上）----------------

    private lateinit var rowMeasure: LinearLayout
    private lateinit var btnAddPoint: Button
    private lateinit var btnBreakChain: Button
    private lateinit var btnUndoPoint: Button
    private lateinit var btnResetPoints: Button

    /**
     * 折线保存在 C++ 侧，因为需要保存 MapPoint*。
     * 这里只保留读取缓冲：每点 3 个 px py flags，每段 5 个 x0 y0 x1 y1 len，
     * 最后 5 个是橡皮筋。
     */
    private var pinnedBuf = FloatArray(8 * 8 + 1)

    /** 上次读回的点数；pinnedBuf.size 是容量，不是点数。 */
    private var pinnedN = 0

    /** 当前折线是否还开着；开着才拉橡皮筋，才允许「断开」。 */
    private val chainOpen: Boolean
        get() = pinnedN > 0 && (pinnedBuf[1 + (pinnedN - 1) * 3 + 2].toInt() and kFlagClosed) == 0

    /** 第 i 段在缓冲中的起始下标。 */
    private fun segAt(i: Int) = 1 + 3 * pinnedN + 5 * i

    /** 段标签缓存：上次数值和文本。 */
    private val segLast = ArrayList<Float>()
    private val segLabels = ArrayList<String>()

    // ---------------- 2D 地图小窗 ----------------

    private lateinit var mapView: MapView

    /** 给 [MapView] 的轨迹缓冲，3 × 512，复用。数据按 stride 抽稀。 */
    private val trajBuf = FloatArray(3 * 512)

    /** 给 [MapView] 的点云缓冲，3 × kMapPointsMax，复用。 */
    private val mapPtBuf = FloatArray(3 * kMapPointsMax)

    /** 小地图容器；开关直接控制其可见性。 */
    private lateinit var mapWrap: FrameLayout
    private lateinit var swMap: SwitchCompat
    private lateinit var sbMapRange: Slider
    private lateinit var tvMapRange: TextView
    private lateinit var btnMapZoomIn: Button
    private lateinit var btnMapZoomOut: Button

    /** 本趟轨迹层；只管画不画，SLAM 始终在记录。 */
    private lateinit var swTraj: SwitchCompat

    /** 上次看到的地图重置次数；增加表示坐标系已更换。 */
    private var resetCountSeen = 0

    /** 缩放键连发定时器，挂在 [uiHandler] 上，onDestroy 时统一清除。 */
    private var zoomRepeat: Runnable? = null

    /** 上次落盘的缩放档位；只在档位变化时写入。 */
    private var zoomSaved = -1

    // ---------------- 路径点 ----------------
    private lateinit var swWaypoints: SwitchCompat
    private lateinit var btnWpAdd: Button
    private lateinit var btnWpManage: Button

    /** 路径点缓冲；wpMeta 每点 4 个：id, mapId, kfId, flags。 */
    private var wpPos = FloatArray(3 * 16)
    private var wpMeta = LongArray(4 * 16)
    private var wpNames = arrayOfNulls<String>(16)
    private var wpN = 0

    /** 上次读取的版本号；变化时才重建图层。 */
    private var wpRevisionSeen = -1L

    /** 上次加路径点的结果，用于仅提示一次。 */
    private var wpStateSeen = 0

    /** 加路径点失败提示。 */
    private var wpMsg: String? = null

    // ---------------- 历史轨迹（小地图上的蓝色层）----------------

    private lateinit var swHistory: SwitchCompat
    private lateinit var btnHistoryManage: Button

    /** nativeDisplayTransforms 的缓冲，每条 9 个 float。 */
    private val histTfBuf = FloatArray(9 * kDisplayTfMax)

    /** 读取单条轨迹的缓冲，3 × kHistoryMaxPts。 */
    private val histPtsBuf = FloatArray(3 * kHistoryMaxPts)

    /** 当前图中存在的全部轨迹，包括隐藏的。 */
    private var histEntries: List<TraceEntry> = emptyList()

    /** 上次重建的上下文；未变化时不访问磁盘。 */
    private var histCtx = ""

    /** 下次需要重建的标记。 */
    private var histDirty = true

    /** 实际绘制的轨迹条数。 */
    private var histShown = 0

    /** 存在的轨迹条数，用于管理按钮状态。 */
    private var histAll = 0

    /** 隐藏轨迹的绝对路径集合，与 pref 同步。 */
    private var histHidden: MutableSet<String> = mutableSetOf()

    /** 一条归档轨迹及其坐标变换（9 个 float）。 */
    private class TraceEntry(val file: File, val mapId: Long, val tf: FloatArray)

    // ---------------- 地图查看器（全屏 2D）----------------

    private lateinit var viewerWrap: FrameLayout
    private lateinit var viewerMapView: MapView
    private lateinit var tvViewerMap: TextView
    private lateinit var btnViewerSlice: Button
    private lateinit var btnMapViewer: Button
    private lateinit var rowStatus: LinearLayout
    private lateinit var rowBottom: LinearLayout

    /** 查看器是否打开；ticker 依赖它继续运行。 */
    private var viewerOpen = false

    /** 当前查看的图 id；-1 表示尚未选择。 */
    private var viewerMapId = -1L

    /** atlas 中关键帧数大于 0 的图。 */
    private data class MapInfo(val id: Long, val kfs: Int, val mps: Int)
    private var viewerMaps: List<MapInfo> = emptyList()

    /** 每张图的已归档轨迹条数。 */
    private var viewerTraceCounts: Map<Long, Int> = emptyMap()

    /** 选中图的点云（3n，该图自身坐标系）。 */
    private var viewerCloud = FloatArray(0)
    private var viewerCloudN = 0
    private val viewerCloudBuf = FloatArray(3 * kMapPointsMax)

    /** 选中图的历史轨迹（每条 3n，使用单位变换）。 */
    private var viewerTraces: List<FloatArray> = emptyList()
    private val viewerTraceBuf = FloatArray(3 * kHistoryMaxPts)

    /** 点云 z 的中位数。 */
    private var viewerMedianZ = 0f

    /** 切片中心模式：false 跟随当前高度，true 使用点云中位数。 */
    private var viewerSliceMedian = false

    /** 地图列表复用缓冲。 */
    private var viewerMapIds = LongArray(32)
    private var viewerMapKfs = IntArray(32)
    private var viewerMapMps = IntArray(32)

    /** 上次刷新地图列表的时刻。 */
    private var viewerListAt = 0L

    // ---------------- 生命周期 ----------------

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        applySystemBarAppearance()

        btnSlam = findViewById(R.id.btnSlam)
        swRecord = findViewById(R.id.swRecord)
        swMeasure = findViewById(R.id.swMeasure)
        tvStatus = findViewById(R.id.tvStatus)
        preview = findViewById(R.id.preview)
        chipTrack = findViewById(R.id.chipTrack)
        chipPoints = findViewById(R.id.chipPoints)
        chipRecord = findViewById(R.id.chipRecord)
        chipMap = findViewById(R.id.chipMap)
        bannerWrap = findViewById(R.id.bannerWrap)
        progress = findViewById(R.id.progress)
        tvDiag = findViewById(R.id.tvDiag)
        btnSlamTint = btnSlam.backgroundTintList
        btnSlamTextColor = btnSlam.currentTextColor

        findViewById<View>(R.id.drawerDiagHeader).setOnClickListener {
            val open = tvDiag.visibility != View.VISIBLE
            tvDiag.visibility = if (open) View.VISIBLE else View.GONE
            findViewById<ImageView>(R.id.drawerDiagChevron).rotation = if (open) 180f else 0f
        }
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
        swTraj = findViewById(R.id.swTraj)
        swHistory = findViewById(R.id.swHistory)
        btnHistoryManage = findViewById(R.id.btnHistoryManage)
        viewerWrap = findViewById(R.id.viewerWrap)
        viewerMapView = findViewById(R.id.viewerMapView)
        tvViewerMap = findViewById(R.id.tvViewerMap)
        btnViewerSlice = findViewById(R.id.btnViewerSlice)
        btnMapViewer = findViewById(R.id.btnMapViewer)
        rowStatus = findViewById(R.id.rowStatus)
        rowBottom = findViewById(R.id.rowBottom)
        drawer = findViewById(R.id.drawer)
        rowMeasure = findViewById(R.id.rowMeasure)
        btnAddPoint = findViewById(R.id.btnAddPoint)
        btnBreakChain = findViewById(R.id.btnBreakChain)
        btnUndoPoint = findViewById(R.id.btnUndoPoint)
        btnResetPoints = findViewById(R.id.btnResetPoints)

        // 在注册监听前恢复开关状态，避免恢复动作触发监听。
        swRecord.isChecked = pref.getBoolean(PREF_RECORD, false)
        // 测距默认关闭。
        swMeasure.isChecked = pref.getBoolean(PREF_MEASURE, false)
        // 同步给 C++，避免两侧状态不一致。
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
        // 开关表示用户意图：启动 SLAM 时若开启则自动录制，运行中拨动立即生效。
        swRecord.setOnCheckedChangeListener { _, checked ->
            pref.edit().putBoolean(PREF_RECORD, checked).apply()
            if (checked) startRecording() else stopRecording()
        }

        // 测距作用在画面上；关闭时清掉叠加层，避免十字残留。
        swMeasure.setOnCheckedChangeListener { _, checked ->
            pref.edit().putBoolean(PREF_MEASURE, checked).apply()
            SlamBridge.nativeSetMeasuring(checked)
            if (!checked) measureState = PreviewView.MeasureState.NO_TRACK
            // 多点折线随开关显示或隐藏；已钉住的点保留。
            render()
        }

        // 小地图开关和切片厚度，同样在注册监听前恢复。
        swMap.isChecked = pref.getBoolean(PREF_MAP, true)
        sbMapRange.value = snapRange(pref.getFloat(PREF_MAP_RANGE, MapView.SLICE_DEFAULT))
        applyMapRange(sbMapRange.value)

        swMap.setOnCheckedChangeListener { _, checked ->
            pref.edit().putBoolean(PREF_MAP, checked).apply()
            applyMapVisible(checked)
        }
        // 先按当前开关同步一次：C++ 那边默认是开的，而用户可能记成了关。
        applyMapVisible(swMap.isChecked)

        // 拖动时立即生效；仅用户操作时写入偏好。
        sbMapRange.addOnChangeListener { _, value, fromUser ->
            applyMapRange(value)
            if (fromUser) pref.edit().putFloat(PREF_MAP_RANGE, value).apply()
        }

        // 缩放档位要记着：上次按到哪一档，这次进来还是哪一档。
        // -1 表示尚未保存，setZoomIndex 会忽略并使用默认档。
        mapView.setZoomIndex(pref.getInt(PREF_MAP_ZOOM, -1))
        zoomSaved = mapView.zoomIndex()
        // 缩放键的长按连发由 wireZoomButton 处理。
        wireZoomButton(btnMapZoomIn) {
            mapView.zoomIn()
            saveZoom()
        }
        wireZoomButton(btnMapZoomOut) {
            mapView.zoomOut()
            saveZoom()
        }

        // 多点折线：加点、断开、撤销、重置。折线保存在 C++ 侧，这里只调用并读取结果。
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

        // 路径点开关只影响是否绘制，管理界面始终可用。
        swWaypoints.isChecked = pref.getBoolean(PREF_WAYPOINTS, true)
        swWaypoints.setOnCheckedChangeListener { _, checked ->
            pref.edit().putBoolean(PREF_WAYPOINTS, checked).apply()
            // 打开时必须强制重读，否则 revision 未变会一直为空。
            wpRevisionSeen = -1
            refreshWaypoints()
        }
        btnWpAdd.setOnClickListener { askAddWaypoint() }
        btnWpManage.setOnClickListener { showWaypointManager() }

        // 实时轨迹：只影响小地图绘制，轨迹本身照常写盘。
        swTraj.isChecked = pref.getBoolean(PREF_MAP_TRAJ, true)
        swTraj.setOnCheckedChangeListener { _, checked ->
            pref.edit().putBoolean(PREF_MAP_TRAJ, checked).apply()
            render()
        }

        // 历史轨迹：开关只管画不画；逐条隐藏 / 删除在管理页（开关关着也能管）。
        histHidden = (pref.getStringSet(PREF_HIST_HIDDEN, emptySet()) ?: emptySet())
            .toMutableSet()
        swHistory.isChecked = pref.getBoolean(PREF_HISTORY, true)
        // 此前 applyMapVisible 已执行过一次，这里标记重建以刷新历史层。
        histDirty = true
        swHistory.setOnCheckedChangeListener { _, checked ->
            pref.edit().putBoolean(PREF_HISTORY, checked).apply()
            histDirty = true
            render()
        }
        btnHistoryManage.setOnClickListener { showHistoryManager() }

        // 地图查看器：全屏第二块 MapView。它不跟当前位置，拖动/捏合自己管。
        viewerMapView.setAutoCenter(false)
        viewerMapView.setContinuousZoom(true)
        viewerMapView.setFullBleed(true)
        viewerSliceMedian = pref.getBoolean(PREF_VIEWER_SLICE, false)
        updateViewerSliceLabel()
        btnMapViewer.setOnClickListener {
            drawer.closeDrawer(GravityCompat.END)
            openMapViewer()
        }
        tvViewerMap.setOnClickListener { showMapListDialog() }
        findViewById<Button>(R.id.btnViewerCurrent).setOnClickListener { jumpToCurrentMap() }
        findViewById<Button>(R.id.btnViewerRefresh).setOnClickListener { reloadViewerMap() }
        findViewById<Button>(R.id.btnViewerFit).setOnClickListener { fitViewer() }
        findViewById<Button>(R.id.btnViewerClose).setOnClickListener { closeMapViewer() }
        findViewById<Button>(R.id.btnViewerZoomIn).setOnClickListener { viewerMapView.zoomIn() }
        findViewById<Button>(R.id.btnViewerZoomOut).setOnClickListener { viewerMapView.zoomOut() }
        btnViewerSlice.setOnClickListener {
            viewerSliceMedian = !viewerSliceMedian
            pref.edit().putBoolean(PREF_VIEWER_SLICE, viewerSliceMedian).apply()
            updateViewerSliceLabel()
            applyViewerSlice()
        }

        // 查看器开着时 Back 先收起它；抽屉开着就先关抽屉（保持原来的手感）。
        onBackPressedDispatcher.addCallback(this, object : OnBackPressedCallback(true) {
            override fun handleOnBackPressed() {
                when {
                    drawer.isDrawerOpen(GravityCompat.END) -> drawer.closeDrawer(GravityCompat.END)
                    viewerOpen -> closeMapViewer()
                    else -> {
                        isEnabled = false
                        onBackPressedDispatcher.onBackPressed()
                        isEnabled = true
                    }
                }
            }
        })

        // 叠加层随画面帧更新，不随 ticker 更新。
        preview.overlaySupplier = { if (swMeasure.isChecked) buildOverlay() else null }

        mapFile = intent.getStringExtra(EXTRA_MAP_FILE)
        Log.i(TAG, "地图来源：" + (mapFile ?: "新建地图"))
        // 载入地图时，会话目录取 .osa 的父目录，这样未启动 SLAM 也能管理历史轨迹。
        mapFile?.let { f ->
            File(f).parentFile?.let { dir ->
                dir.mkdirs()
                sessionDir = dir
            }
        }

        // 读取构造状态；System 是进程级的，可能由上一个 Activity 实例创建。
        // 这里也是第一次访问 SlamBridge，.so 在此刻加载并由 JNI_OnLoad 安装日志重定向。
        slamState = SlamBridge.nativeState()

        // System 已存在时，入口页的选择需要手动换源，nativeInit 不会再次执行。
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
        // 切后台等同于停止；返回后需要重新启动。
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
    // 没有取消准备：System 构造无法中途取消，也没有安全的销毁时机。

    private fun onButtonSlam() {
        // 上一次的错误提示只留到下一次操作。
        uiMessage = null

        // 等对焦中：用户觉得清楚了就立刻锁定并开始，不必等自动收敛。
        if (waitingFocus) {
            cam.lockFocusNow()
            finishStart()
            render()
            return
        }

        // 运行中：结束本次会话
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

        // System 已就绪：只打开硬件，不重新加载
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
     * 打开相机和 IMU，并调用 nativeStart()。必须在 System 就绪后执行。
     */
    private fun openHardware() {
        if (slamRunning || !hasCameraPermission()) return

        // IMU 先起步（本机从注册到出数要 3 秒多，正好和相机启动重叠）。
        val t = HandlerThread("imu").also { it.start() }
        val c = ImuCollector(this)
        imuThread = t
        imu = c
        c.onSample = { ts, wx, wy, wz, ax, ay, az ->
            // 运行在 IMU 线程上，不能阻塞。
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

        // 内参以地图记录为准，镜组也回到记录的那个屈光度。
        // 既包括从入口页打开的旧图，也包括本页已经跑过一趟的新图（sessionDir 已建）。
        val recorded = (mapFile?.let { File(it).parentFile } ?: sessionDir)
            ?.let { CalibStore.readMapIntrinsics(it) }
        if (recorded != null) {
            cam.pinnedDiopter = recorded.d
            cam.start()
            cam.beginSettle()
            applyIntrinsics(recorded, "地图记录")
            startTracking()
            return
        }

        // 新地图：先让用户对好焦，收敛后再按屈光度查表。等对焦由 ticker 推进。
        //
        // 走到这里还可能是"打开了一张没有内参记录的老图"：那就按当前对焦取一档，
        // 开跑后把它记进这张图，之后就一直用它。
        if (mapFile != null) uiMessage = "这张图没有内参记录，按当前对焦取一档"
        cam.pinnedDiopter = null
        val rows = CalibStore.load(this)?.rows.orEmpty()
        // 收敛到表外就只提示、不锁：锁下去整段画面都会糊。
        // 没有标定表时用 2.5 D（0.4 m）兜底 —— 锁到更近的位置整段远景都会糊。
        cam.autofocusMaxD = if (rows.isEmpty()) 2.5f else rows.maxOf { it.d } + 1.0f
        cam.start()
        cam.beginSettle()
        waitingFocus = true
        focusDeadlineMs = SystemClock.elapsedRealtime() + FOCUS_WAIT_MS
        render()
        uiQueueTicker()
    }

    /**
     * 对焦收敛（或超时）后：定内参 → 注入 → 真正开始吃帧。
     * 用户点「锁定并开始」时也会直接走这里。
     */
    private fun finishStart() {
        if (!waitingFocus) return
        waitingFocus = false

        // 超出标定范围时 focusDiopter 是空的、镜组还在连续对焦：这时**不要**去锁
        // （锁在 0.2 m 上整段远景都糊），按默认档取内参，画面保持 AF 当前的样子。
        val outOfRange = cam.focusDiopter == null && cam.focusNote != null
        // 超时自动开跑时可能还没锁：能读到当前屈光度就锁住，否则内参会随场景漂。
        // 定焦模组没有这个概念，不报警。
        if (!outOfRange && cam.focusDiopter == null &&
            cam.lockFocusNow() == null && cam.supportsAf
        ) {
            uiMessage = "读不到对焦距离 —— 内参会随画面变化，结果不可比"
        }

        val d = if (outOfRange) DEFAULT_DIOPTER
                else cam.focusDiopter ?: cam.latestFocusDistance() ?: DEFAULT_DIOPTER
        val rows = CalibStore.load(this)?.rows.orEmpty()
        val row = CalibStore.interpolate(rows, d)

        if (row != null) {
            applyIntrinsics(row, "档位插值")
            val gap = CalibStore.distanceToTable(rows, d)
            if (gap > 1.0f) {
                uiMessage = "对焦 %.2f D 离最近的标定档差 %.2f D，内参是外推的".format(d, gap)
            }
        } else {
            val fb = CalibStore.fallback(this)
            if (fb != null) {
                // 兜底值没有档位，但这一趟确实是在 d 上跑的；记录要用真实的 d，
                // 否则下次打开这张图会把镜组命令到一个没测过的位置。
                applyIntrinsics(fb.copy(d = d), "模板兜底")
                uiMessage = "还没有标定表，用的是模板内参 —— 换设备后不要信这组数"
            } else {
                intrinsicsNote = "未注入（读不到任何内参）"
            }
        }
        startTracking()
    }

    /** 把内参交给 C++。必须在 [startTracking] 之前 —— 帧一喂进去上游就开始用它。 */
    private fun applyIntrinsics(row: CalibRow, from: String) {
        activeRow = row
        intrinsicsNote = "%s · %.2f D · fx %.1f".format(
            CalibStore.sourceLabel(row), row.d, row.fx)
        Log.i(TAG, "内参（$from）：fx=%.3f fy=%.3f cx=%.3f cy=%.3f k1=%.5f k2=%.5f d=%.2f"
            .format(row.fx, row.fy, row.cx, row.cy, row.k1, row.k2, row.d))
        SlamBridge.nativeSetIntrinsics(
            row.fx, row.fy, row.cx, row.cy, row.k1, row.k2, row.p1, row.p2)
    }

    /** nativeStart() 以及本趟的会话目录/录制初始化。内参已经定好才会走到这里。 */
    private fun startTracking() {
        SlamBridge.nativeStart()   // 顺带把 C++ 那边的轨迹和重置计数清零
        slamRunning = true
        trkMsg = null
        trkMsgUntil = 0L

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
        // 一张图一套内参：没有记录就补上，之后每次打开这张图都用它。
        sessionDir?.let { dir ->
            val row = activeRow
            if (row != null && CalibStore.readMapIntrinsics(dir) == null) {
                CalibStore.writeMapIntrinsics(dir, row, Build.MODEL, cameraId)
            }
        }

        // 一趟固定一个时间戳；轨迹先落 traj/，存图成功才归档到 map-<id>/。
        runStamp = stamp()
        curTrajBase = null
        curRecDir = null
        runMapId = -1
        runMerges = emptyList()
        histDirty = true   // 新一趟：显示用的图 id / 合并链都会变
        if (swRecord.isChecked) startRecording()

        Log.i(TAG, "SLAM 开始吃帧")
        // 重新启动 ticker 以刷新位姿。
        uiQueueTicker()
    }

    /**
     * 关闭硬件，幂等，由停止、切后台和退出共用。
     * 不销毁 System；C++ 的 stop() 只停止吃帧并清空队列。
     */
    private fun stopHardware(reason: String) {
        // 还在等对焦、没真正开跑：只拆硬件，不碰会话文件。
        if (waitingFocus) {
            waitingFocus = false
            preview.setActive(false)
            imu?.stop()
            imuThread?.quitSafely()
            imu = null
            imuThread = null
            cam.stop()
            render()
            return
        }
        if (!slamRunning) return
        Log.i(TAG, "SLAM 停止吃帧（$reason）")

        SlamBridge.nativeStop()   // 先让它别吃了，再拆硬件

        // 紧跟 nativeStop() 读取快照；再晚可能已被恢复请求换掉。
        runMapId = SlamBridge.nativeLastTrajMapId()
        runMerges = readMerges()
        slamRunning = false

        stopRecording()           // 正在录的话文件一律保留（录到一半也是真数据）

        // 轨迹先写入 traj/，存图成功后才归档到 map-<id>/。
        // 仅主动结束时询问是否保留；切后台或退出时直接保留。
        val base = newTrajBase()
        if (base != null) {
            curTrajBase = base
            exportTrajectory(base)
            // 元信息立即写入，避免归档时两个文件分处不同目录。
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
        // 只清除画面；叠加层随后续帧更新，无帧时不会显示。
        preview.setActive(false)
        measureState = PreviewView.MeasureState.NO_TRACK
        // ticker 停止后补充刷新一次，保留本趟最终画面。
        // 先刷新历史，因为 hint 依赖当前是否有内容可画。
        refreshHistory()
        refreshMap()
    }

    // ---------------- 帧出口 ----------------

    /**
     * 每帧出口，运行在相机线程上，不能阻塞。
     * 三个消费者各自检查并捕获异常，互不影响。
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

        // 本趟录像目录为 rec/<runStamp>/；中途开始录制也使用同一目录。
        val run = curRecDir
            ?: File(TrajectoryStore.recDir(session), runStamp).also { curRecDir = it }

        // 每段使用独立目录；复用目录会截断 CSV，导致与已有 PNG 不匹配。
        val dir = nextSessionDir(run)
        val w = SessionWriter(dir)
        // 标记为 SLAM 期间录制；格式与录制模式相同，但帧率受落盘影响。
        w.withSlam = true
        // 内参也落一份，事后才能知道这段数据是在什么条件下采的。
        activeRow?.let { r ->
            w.calib = SessionWriter.CalibSnapshot(
                focusMode = if (cam.focusCommanded) "locked" else "auto",
                diopter = cam.focusDiopter ?: r.d,
                source = r.source,
                fx = r.fx, fy = r.fy, cx = r.cx, cy = r.cy, k1 = r.k1,
            )
        }
        w.start()
        if (w.ioError != null) {
            Log.e(TAG, "建 session 文件失败：${w.ioError}")
            uiMessage = "无法创建输出文件：${w.ioError}\n$dir"
            swRecord.isChecked = false   // 写不进去就别让开关骗人
            render()
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

    /** base 不含扩展名；轨迹写 base.txt，元信息写 base.json。 */
    private fun exportTrajectory(base: File) {
        try {
            SlamBridge.nativeDumpTrajectory("$base.txt")
        } catch (e: Throwable) {
            Log.e(TAG, "导出轨迹失败", e)
        }
    }

    // ---------------- 状态栏 ----------------

    /**
     * 加载中和运行中都需要运行：
     * 前者刷新加载进度，后者刷新位姿和跟踪状态。
     */
    private val ticker = object : Runnable {
        override fun run() {
            if (slamState == SLAM_LOADING) {
                slamState = SlamBridge.nativeState()
                slamElapsedMs = SlamBridge.nativeElapsedMs()
                // 不在这里打开硬件；就绪和启动是两个阶段。
            }
            if (slamRunning) {
                poseValid = SlamBridge.nativeGetPose(poseBuf)
                trkState = SlamBridge.nativeGetTrackingState()
                pollMeasure()
            }

            // 等对焦：有结论（收敛 / 判定锁不了）或超时就定内参、真正开跑。
            if (waitingFocus &&
                (cam.ready || SystemClock.elapsedRealtime() >= focusDeadlineMs)
            ) {
                finishStart()
            }

            // 先刷新界面，再判断是否继续调度；否则最后一轮刷新会被跳过。
            render()

            if (slamState == SLAM_LOADING || slamRunning || waitingFocus ||
                saving || restoring || viewerOpen
            ) {
                uiHandler.postDelayed(this, TICK_MS)
            }
        }
    }

    private fun uiQueueTicker() {
        uiHandler.removeCallbacks(ticker)
        uiHandler.post(ticker)
    }

    /**
     * 读取一次画面中心的距离读数，交给 [PreviewView] 绘制。
     * 跟踪未就绪单独一档，避免被误显示为「对准有纹理的地方」。
     */
    private fun pollMeasure() {
        // 这里只更新状态；叠加层由 overlaySupplier 随新帧生成。
        if (!swMeasure.isChecked) return

        // 地图重置后坐标系重新锚定，已钉住的点失效，需要清空。
        // 使用 > 而不是 !=，因为 nativeStart() 会把计数归零。
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
     * 读取一次测量并更新 [measureState]，返回 C++ 的质量码。
     * ticker 和 buildOverlay 都会调用，分别负责按钮状态和绘制。
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
     * 折线变化后重新读取并刷新按钮状态。
     * 叠加层会随下一帧自动更新。
     */
    private fun refreshPinned() {
        readPinned()
        render()
    }

    /**
     * 读取折线到 [pinnedBuf]，点数写入 [pinnedN]。主线程。
     *
     * C++ 在容量不足时只写装得下的部分，因此缓冲区保留 8 个点余量；
     * 余量不足时提前扩容，避免正常使用中出现截断。
     */
    private fun readPinned(): Int {
        pinnedN = SlamBridge.nativeGetPinned(pinnedBuf)
        // 余量低于 kPinnedSlack 时提前扩容；与现有大小一起翻倍，减少重分配。
        val cap = (pinnedBuf.size - 1) / 8   // C++ 最多能写几个点（那个 -1 留给橡皮筋那块）
        if (cap - pinnedN < kPinnedSlack) {
            pinnedBuf = FloatArray(maxOf(8 * (pinnedN + kPinnedSlack) + 1, pinnedBuf.size * 2))
        }
        return pinnedN
    }

    /**
     * 生成段标签，只在数值变化超过 5 mm 时重新格式化。
     * 长度每帧都会变化，但不应每帧调用 String.format；
     * 显示到厘米 —— 毫米位放大的只是地图收敛的抖动。
     */
    private fun segLabel(i: Int, v: Float): String {
        while (segLast.size <= i) {
            segLast.add(-1f)
            segLabels.add("")
        }
        if (segLast[i] < 0f || abs(v - segLast[i]) > 5e-3f) {
            segLast[i] = v
            segLabels[i] = "%.2f m".format(v)
        }
        return segLabels[i]
    }

    private fun buildOverlay(): PreviewView.Overlay {
        // 这里也需要读取测量；只靠 4Hz 的 ticker 会导致读数、吸附圈和橡皮筋跳动。
        readMeasure()
        val n = readPinned()
        val marks = ArrayList<PreviewView.Mark>(n)
        val segs = ArrayList<PreviewView.Seg>(maxOf(0, n - 1))
        var live: PreviewView.Seg? = null

        // 每点 3 个：px py flags；只绘制相机前方的标记点。
        for (i in 0 until n) {
            val o = 1 + i * 3
            val f = pinnedBuf[o + 2].toInt()
            if (f and kFlagFront == 0) continue
            marks.add(PreviewView.Mark(pinnedBuf[o], pinnedBuf[o + 1], f and kFlagLow != 0))
        }
        // 每段 5 个：x0 y0 x1 y1 len；端点由 C++ 计算，这里只检查 len。
        for (i in 0 until n - 1) {
            val o = segAt(i)
            if (pinnedBuf[o + 4] < 0f) continue
            segs.add(PreviewView.Seg(pinnedBuf[o], pinnedBuf[o + 1], pinnedBuf[o + 2], pinnedBuf[o + 3],
                                     segLabel(i, pinnedBuf[o + 4])))
        }
        // 橡皮筋：最后一个点到十字；断开、无读数或两端在相机后方时为 -1。
        if (n > 0) {
            val lo = segAt(n - 1)
            if (pinnedBuf[lo + 4] >= 0f) {
                live = PreviewView.Seg(pinnedBuf[lo], pinnedBuf[lo + 1],
                                       pinnedBuf[lo + 2], pinnedBuf[lo + 3],
                                       "%.2f m".format(pinnedBuf[lo + 4]))
            }
        }
        return PreviewView.Overlay(measureState, measureBuf[0], measureBuf[4], measureBuf[5],
                                   marks, segs, live)
    }

    /**
     * 把当前读数钉成一个点。
     *
     * 先用上一帧缓冲的投影判断是否吸附到已有点；用户依据屏幕位置判断是否为同一点。
     * 命中后交给 C++ 复制该点的身份，而不是写入当前读数命中的另一个地图点。
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
     * 断开当前折线：保留已有点，只停止橡皮筋，下一个点另起一条。
     * 折线状态完全由读回的 flags 推导。
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
        // 加载中和构造失败时不可点；恢复期间也不可点。
        btnSlam.isEnabled =
            (slamRunning || slamState == SLAM_IDLE || slamState == SLAM_READY) && !restoring
        btnSlam.text = when {
            // 加载中 / 构造失败优先：换源请求这会儿还没轮到（失败的话永远不会轮到）。
            slamState == SLAM_LOADING -> "加载中…"
            slamState == SLAM_FAILED -> "构造失败"
            restoring -> "载入地图…"
            slamRunning -> "结束"
            waitingFocus -> "锁定并开始"
            slamState == SLAM_READY -> "启动 SLAM"
            else -> "准备 SLAM"
        }
        // 「运行中」用错误色提示这一下会结束会话；其余状态用主题默认填充色。
        if (slamRunning) {
            btnSlam.backgroundTintList =
                ColorStateList.valueOf(materialColor(MaterialR.attr.colorError))
            btnSlam.setTextColor(materialColor(MaterialR.attr.colorOnError))
        } else {
            btnSlam.backgroundTintList = btnSlamTint
            btnSlam.setTextColor(btnSlamTextColor)
        }

        // 两个开关始终可点，因为需要在启动前就能设置。
        swRecord.isEnabled = true
        swMeasure.isEnabled = true

        // ---- 多点折线：随测距开关显示或隐藏；查看器打开时也隐藏 ----
        rowMeasure.visibility =
            if (!viewerOpen && swMeasure.isChecked) View.VISIBLE else View.GONE
        // 无有效读数时禁用「加点」，避免用户误以为功能损坏。
        btnAddPoint.isEnabled = swMeasure.isChecked && measureState.hasReading
        // 折线未开时禁用「断开」。
        btnBreakChain.isEnabled = swMeasure.isChecked && chainOpen
        btnUndoPoint.isEnabled = pinnedN > 0
        btnResetPoints.isEnabled = pinnedN > 0

        // 放路径点：跑着且有位姿才可点。
        btnWpAdd.isEnabled = slamRunning && poseValid
        btnWpManage.isEnabled = SlamBridge.nativeWaypointCount() > 0

        // 加路径点结果只提示一次；wpState 会保持为 2 或 3。
        val wpState = SlamBridge.nativeWaypointState()
        if (wpState != wpStateSeen) {
            wpStateSeen = wpState
            when (wpState) {
                3 -> wpMsg = "放路径点失败：${SlamBridge.nativeWaypointMsg()}"
                2 -> wpMsg = null
            }
        }

        // 存图状态转成人话。文件名不用问 C++ —— 就是这一侧传过去的。
        when (SlamBridge.nativeSaveState()) {
            1, 2 -> saveMsg = "正在保存地图…"
            3 -> {
                // 只在保存中变为完成的那一帧归档；saveState 会保持为 3，否则会重复变换坐标。
                val justFinished = saving
                saveMsg = "地图已保存 → ${savedName ?: "?"}.osa"
                saving = false
                if (justFinished) {
                    applyPendingSave()
                    // 文件夹内容变了（归档 / 合并重组）。脏标记放调用点：
                    // applyPendingSave 有好几条提前 return 的路径。
                    histDirty = true
                    // 存完就停下 ticker 了，这条提示要自己安排一次收起。
                    transientMsg = saveMsg
                    transientUntil = SystemClock.elapsedRealtime() + 5000
                    uiHandler.removeCallbacks(transientClear)
                    uiHandler.postDelayed(transientClear, 5200)
                }
            }
            4 -> {
                saveMsg = "地图保存失败（看 logcat 的 slam）"
                saving = false
                pendingArchive = null   // 磁盘不动，松文件留在 traj/
            }
        }

        // ---- 状态芯片 ----
        val mapLabel = mapFile?.let { File(it).name } ?: "新建地图"
        val trkLabel = when {
            !slamRunning -> "未运行"
            trkState == TRK_OK -> "跟踪中"
            trkState == TRK_RECENTLY_LOST -> "跟踪不稳"
            trkState == TRK_LOST -> "已丢失"
            trkState == TRK_NOT_INITIALIZED -> "初始化中"
            trkState == TRK_NO_IMAGES -> "等画面"
            else -> "未就绪"
        }
        val trkColor = when {
            !slamRunning -> materialColor(MaterialR.attr.colorOnSurfaceVariant)
            trkState == TRK_OK -> materialColor(MaterialR.attr.colorPrimary)
            trkState == TRK_LOST -> materialColor(MaterialR.attr.colorError)
            trkState == TRK_RECENTLY_LOST || trkState == TRK_NOT_INITIALIZED ->
                materialColor(MaterialR.attr.colorTertiary)
            else -> materialColor(MaterialR.attr.colorOnSurfaceVariant)
        }
        chipTrack.text = "● $trkLabel"
        chipTrack.setTextColor(trkColor)

        chipPoints.visibility = if (slamRunning) View.VISIBLE else View.GONE
        chipPoints.text = "轨迹 ${SlamBridge.nativeTrajSize()} 点"

        val w = writer
        if (recording && w != null) {
            chipRecord.visibility = View.VISIBLE
            chipRecord.text = "● 录制 ${w.framesWritten.get()}"
        } else {
            chipRecord.visibility = View.GONE
        }

        chipMap.text = mapLabel

        // ---- 横幅：只放需要说明的事 ----
        val now = SystemClock.elapsedRealtime()
        val trkLine = if (slamRunning && trkState != TRK_OK) trackingLine() else null
        if (trkLine != null) {
            trkMsg = trkLine
            trkMsgUntil = now + 2000
        } else if (now >= trkMsgUntil) {
            trkMsg = null
        }
        val banner = when {
            uiMessage != null -> uiMessage
            transientMsg != null && now < transientUntil -> transientMsg
            wpMsg != null -> wpMsg
            slamState == SLAM_LOADING ->
                "词袋加载中…已 ${slamElapsedMs / 1000} 秒"
            slamState == SLAM_FAILED ->
                if (SlamBridge.nativeFailureCode() == 1) {
                    "载图失败 —— 这张图读不了，回入口页换一张或新建"
                } else {
                    "构造失败 —— 看 logcat（tag slam）"
                }
            restoring -> restoreMsg ?: "正在载入地图…"
            waitingFocus -> cam.focusDiopter?.let {
                "对焦 %.2f D · 点「锁定并开始」".format(it)
            } ?: "对焦中… 点「锁定并开始」可立刻锁定"
            !slamRunning && slamState == SLAM_READY ->
                "已就绪 · 点「启动 SLAM」打开相机和 IMU"
            !slamRunning -> "点「准备 SLAM」构造 System"
            trkLine != null -> trkLine
            now < trkMsgUntil -> trkMsg
            else -> null
        }
        // 查看器盖住预览时也收起：它有 elevation，不能只靠绘制顺序遮。
        bannerWrap.visibility =
            if (viewerOpen || banner == null) View.GONE else View.VISIBLE
        tvStatus.text = banner ?: ""
        progress.visibility =
            if (slamState == SLAM_LOADING || restoring || saving) View.VISIBLE else View.GONE

        // ---- 诊断面板：数字和路径都在这儿，平时收着 ----
        tvDiag.text = buildString {
            append("地图来源  $mapLabel\n")
            // 内参只有一个真相：profile / 地图记录。这里显示的是这一趟实际注入的那套。
            append("内参  $intrinsicsNote\n")
            val fd = cam.focusDiopter
            if (fd != null) {
                append("对焦  %.2f D".format(fd))
                cam.focusNote?.let { append("  · $it") }
                append('\n')
            } else {
                cam.focusNote?.let { append("对焦  $it\n") }
            }
            if (slamRunning && poseValid) {
                append("位置  %.3f  %.3f  %.3f\n".format(poseBuf[12], poseBuf[13], poseBuf[14]))
            }
            if (slamRunning) {
                append("轨迹  ${SlamBridge.nativeTrajSize()} 点")
                val resets = SlamBridge.nativeResetCount()
                if (resets > 0) append(" · 重置 $resets 次")
                append('\n')
                // 十字附近的特征点数；无读数时也有值，可用于说明测不到的原因。
                if (swMeasure.isChecked) append("纹理  %.0f\n".format(measureBuf[6]))
            }
            if (recording && w != null) {
                append("录制  相机 ${w.framesWritten.get()} 帧 / IMU ${w.imuWritten.get()} 条\n")
            }
            // 地图统计；恢复期间不读取，因为 atlas 正在重建。
            if (slamState == SLAM_READY && !restoring) {
                SlamBridge.nativeAtlasStats(atlasBuf)
                append("地图  ${atlasBuf[0]} 张 · KF ${atlasBuf[1]} · 点 ${atlasBuf[2]}")
                val wpTotal = SlamBridge.nativeWaypointCount()
                if (wpTotal > 0) append(" · 路径点 $wpTotal")
                append('\n')
            }
            restoreMsg?.let { append(it).append('\n') }
            saveMsg?.let { append(it).append('\n') }
            wpMsg?.let { append(it).append('\n') }
            lastRunDir?.let { append("上一趟  ${it.absolutePath}") }
        }.trimEnd()

        // 2D 地图小窗也由这儿喂 —— 于是 ticker、按钮事件、以及 idle 的首帧全覆盖。
        // 恢复期间跳过：轨迹表刚被清、点云快照也刚被清，读出来是一片空。
        // 先刷新历史，refreshMap 的提示依赖历史数量。
        if (!restoring) {
            refreshHistory()
            refreshMap()
            refreshWaypoints()
        }
        if (viewerOpen) refreshMapViewer()
    }

    // ---------------- 路径点 ----------------

    /**
     * 读取路径点并更新小地图。
     *
     * 不能仅按 revision 跳过：回环或合并会改变坐标而不改变 revision。
     * 只向 [MapView] 传入有效且位于当前图的点。
     */
    private fun refreshWaypoints() {
        // 小地图关着、查看器也没开：不用读。
        if (!swWaypoints.isChecked && !viewerOpen) {
            mapView.clearWaypoints()
            return
        }

        // 开关关闭但查看器打开时仍需读取；查看器按 mapId 筛选自己的图。
        readWaypointList()

        if (!swWaypoints.isChecked) {
            mapView.clearWaypoints()
        } else {
            // 小地图：只画「有效 + 在当前图」的；别的图在另一个坐标系里。
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

        // 查看器绘制选中图中的有效路径点，不要求是当前图。
        if (viewerOpen) {
            val xs = FloatArray(wpN * 2)
            val ns = arrayOfNulls<String>(wpN)
            var k = 0
            for (i in 0 until wpN) {
                val flags = wpMeta[i * 4 + 3]
                if ((flags and kFlagWpValid) == 0L) continue
                if (wpMeta[i * 4 + 1] != viewerMapId) continue
                xs[k * 2] = wpPos[i * 3]
                xs[k * 2 + 1] = wpPos[i * 3 + 1]
                ns[k] = wpNames[i]
                k++
            }
            viewerMapView.setWaypoints(xs, k, ns)
        }
    }

    /** 读取路径点到缓冲；装不下时扩容重读。 */
    private fun readWaypointList() {
        while (true) {
            val n = SlamBridge.nativeGetWaypoints(wpPos, wpMeta, wpNames)
            if (n < wpPos.size / 3 && n < wpMeta.size / 4 && n < wpNames.size) {
                wpN = n
                return
            }
            wpPos = FloatArray(wpPos.size * 2)
            wpMeta = LongArray(wpMeta.size * 2)
            wpNames = arrayOfNulls(wpNames.size * 2)
        }
    }

    /** 添加路径点：先询问名字，再发送请求。 */
    private fun askAddWaypoint() {
        // 使用下一个 id 而不是条数，避免删除后重名。
        textInputDialog("添加路径点", "路径点 ${SlamBridge.nativeNextWaypointId()}") { name ->
            val n = name.ifEmpty { "路径点" }
            Log.i(TAG, "加路径点「$n」")
            SlamBridge.nativeRequestAddWaypoint(n)
            wpStateSeen = 1
            uiQueueTicker()
        }
    }

    /** 路径点管理界面，包含不在当前图和已失效的条目。 */
    private fun showWaypointManager() {
        val total = SlamBridge.nativeWaypointCount()
        if (total == 0) {
            MaterialAlertDialogBuilder(this)
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

        MaterialAlertDialogBuilder(this)
            .setTitle("路径点（$n）")
            .setItems(labels) { _, which ->
                val id = meta[which * 4].toInt()
                val name = names[which] ?: "路径点"
                MaterialAlertDialogBuilder(this)
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
        textInputDialog("重命名", old) { name ->
            SlamBridge.nativeRenameWaypoint(id, name.ifEmpty { old })
            refreshWaypoints()
            render()
        }
    }

    // ---------------- 2D 地图小窗 ----------------

    /**
     * 读取轨迹和点云并交给小地图；计算和绘制由 [MapView] 完成。
     * 小地图隐藏时不读取，避免无谓的加锁。
     */
    private fun refreshMap() {
        // 查看器盖住了小窗：不读、不画，省一倍锁和绘制。
        if (mapWrap.visibility != View.VISIBLE || viewerOpen) return

        // 轨迹层关掉时不读 native：省一次轨迹锁。
        val n = if (swTraj.isChecked) SlamBridge.nativeGetTrajectory(trajBuf) else 0
        val pn = SlamBridge.nativeGetMapPoints(mapPtBuf)
        // 空窗也要给出原因，并区分加载中、未启动等状态。
        // 只在无任何可绘制内容时才显示提示；主动隐藏轨迹不算「没内容」。
        val anything = n > 0 || pn > 0 || histShown > 0
        val hint = when {
            slamState == SLAM_LOADING -> "词袋加载中…"
            slamState == SLAM_FAILED -> "构造失败"
            !anything && !slamRunning -> "SLAM 未启动"
            !anything && trkState != TRK_OK -> "跟踪未就绪"
            !anything && swTraj.isChecked -> "等待定位…"
            else -> null
        }
        // 停止后不绘制当前位置，避免使用过期位姿。
        val hasCur = slamRunning && poseValid
        // 朝向：将相机光轴投影到 XY 平面。poseBuf 是列主序 Twc，
        // 第 3 列即相机 +z 轴的世界方向；只排除正下方的情况。
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
     * 读取当前图的历史轨迹并交给小地图。
     * 只读取该图对应的 map-<id>/ 目录；合并变换已由 nativeDisplayTransforms 合成。
     * 仅在上下文变化或标记为脏时访问磁盘。
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
        // 单点轨迹无法绘制（MapView 至少需要 2 点），不计入提示判据。
        histShown = traces.count { it.size >= 6 }
        mapView.setHistory(traces)
    }

    /** 管理当前图的已归档轨迹：显示、隐藏、删除。 */
    private fun showHistoryManager() {
        // 重新算一次条目：用户可能刚在别处删过 / 加过。
        histDirty = true
        refreshHistory()
        val entries = histEntries
        if (entries.isEmpty()) {
            MaterialAlertDialogBuilder(this)
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
        MaterialAlertDialogBuilder(this)
            .setTitle("历史轨迹（${entries.size}）")
            .setItems(labels) { _, which -> showTraceActions(entries[which]) }
            .setNegativeButton("关闭", null)
            .show()
    }

    private fun showTraceActions(e: TraceEntry) {
        val hidden = e.file.absolutePath in histHidden
        MaterialAlertDialogBuilder(this)
            .setTitle(e.file.name)
            .setItems(arrayOf(if (hidden) "显示" else "隐藏", "删除")) { _, pick ->
                if (pick == 0) setTraceHidden(e.file, !hidden) else askDeleteTrace(e.file)
            }
            .setNegativeButton("取消", null)
            .show()
    }

    private fun setTraceHidden(f: File, hidden: Boolean) {
        if (hidden) histHidden.add(f.absolutePath) else histHidden.remove(f.absolutePath)
        // 不能修改 getStringSet 返回的对象，需存入新集合。
        pref.edit().putStringSet(PREF_HIST_HIDDEN, histHidden.toSet()).apply()
        histDirty = true
        render()
    }

    private fun askDeleteTrace(f: File) {
        MaterialAlertDialogBuilder(this)
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

    /** 读取轨迹元信息中的点数；读不到时返回 -1。 */
    private fun readTracePoints(txt: File): Int = try {
        val meta = File(txt.absolutePath.removeSuffix(".txt") + ".json")
        if (!meta.isFile) -1
        else Regex("\"traj_points\"\\s*:\\s*(\\d+)").find(meta.readText())
            ?.groupValues?.get(1)?.toInt() ?: -1
    } catch (e: Exception) {
        -1
    }

    // ---------------- 地图查看器 ----------------

    private fun openMapViewer() {
        if (viewerOpen) return
        when {
            restoring -> { viewerDialog("正在载入地图…"); return }
            slamState == SLAM_IDLE -> {
                viewerDialog("先点「准备 SLAM」，构造出 atlas 之后才能看地图。")
                return
            }
            slamState == SLAM_LOADING -> {
                viewerDialog("词库加载中，等就绪后再看。")
                return
            }
            slamState == SLAM_FAILED -> {
                viewerDialog("SLAM 构造失败，没有可看的 atlas。")
                return
            }
        }

        viewerOpen = true
        rowStatus.visibility = View.GONE
        rowBottom.visibility = View.GONE
        rowMeasure.visibility = View.GONE
        viewerWrap.visibility = View.VISIBLE
        viewerMapView.setSliceRange(sbMapRange.value)
        applyViewerSlice()
        updateViewerSliceLabel()

        refreshViewerList()
        if (viewerMaps.isEmpty()) {
            setViewerHint("atlas 里还没有图")
            tvViewerMap.text = "地图 · atlas 里还没有图"
        } else {
            selectViewerMap(preferredViewerMapId())
        }
        viewerListAt = SystemClock.elapsedRealtime()
        uiQueueTicker()
        render()
    }

    private fun closeMapViewer() {
        if (!viewerOpen) return
        viewerOpen = false
        viewerWrap.visibility = View.GONE
        rowStatus.visibility = View.VISIBLE
        rowBottom.visibility = View.VISIBLE
        // rowMeasure 由 render 按「实时测距」开关恢复。
        render()
    }

    /** 查看器打开时每帧调用：刷新列表、位置和切片；点云仅在选图或刷新时读取。 */
    private fun refreshMapViewer() {
        if (restoring) {
            setViewerHint("正在载入地图…")
            return
        }

        val now = SystemClock.elapsedRealtime()
        if (now - viewerListAt >= 2000L) {
            viewerListAt = now
            refreshViewerList()
        }

        if (viewerMaps.isEmpty()) {
            val msg = when (slamState) {
                SLAM_IDLE -> "先准备 SLAM"
                SLAM_LOADING -> "词库加载中…"
                SLAM_FAILED -> "构造失败"
                else -> "atlas 里还没有图"
            }
            setViewerHint(msg)
            tvViewerMap.text = "地图 · $msg"
            return
        }

        // 选中的图被合并/换图弄没了：切到当前图（没有就切到最大 id）。
        if (viewerMaps.none { it.id == viewerMapId }) {
            val id = preferredViewerMapId()
            if (id >= 0) selectViewerMap(id)
            return
        }

        val trajId = SlamBridge.nativeTrajectoryMapId()
        val onCurrent = viewerMapId == trajId
        val n = if (onCurrent) SlamBridge.nativeGetTrajectory(trajBuf) else 0
        val trackingNow = slamRunning && poseValid && trkState == TRK_OK
        val hasCur = onCurrent && trackingNow

        // 高度模式：小地图被盖住不重绘，这里把当前高度喂给它那套低通；
        // 看别的图时也照样跟着人走（「按小地图当前高度」的口径）。
        val heightZ = if (trackingNow) mapView.trackSliceCenter(poseBuf[14])
                      else mapView.sliceCenter()
        viewerMapView.setSliceCenter(if (viewerSliceMedian) viewerMedianZ else heightZ)

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

        val hasData = viewerCloudN > 0 || viewerTraces.isNotEmpty() || n > 0
        val hint = if (hasData) null else "这张图没有可显示的数据"
        viewerMapView.submit(MapView.Snapshot(
            trajBuf, n,
            if (hasCur) poseBuf[12] else 0f,
            if (hasCur) poseBuf[13] else 0f,
            if (hasCur) poseBuf[14] else 0f,
            hasCur, hint, dx, dy,
            viewerCloud, viewerCloudN))
        updateViewerTitle(trajId)
    }

    /** 重新读取 atlas 的地图列表，约每 2 秒一次。 */
    private fun refreshViewerList() {
        if (restoring || slamState != SLAM_READY) {
            if (viewerMaps.isNotEmpty()) {
                viewerMaps = emptyList()
                viewerTraceCounts = emptyMap()
            }
            return
        }

        // 看到写满就扩容重读，与 nativeGetWaypoints 契约相同。
        var n: Int
        while (true) {
            n = SlamBridge.nativeMapList(viewerMapIds, viewerMapKfs, viewerMapMps)
            if (n < viewerMapIds.size) break
            viewerMapIds = LongArray(viewerMapIds.size * 2)
            viewerMapKfs = IntArray(viewerMapKfs.size * 2)
            viewerMapMps = IntArray(viewerMapMps.size * 2)
        }

        val list = ArrayList<MapInfo>(n)
        for (i in 0 until n) {
            list += MapInfo(viewerMapIds[i], viewerMapKfs[i], viewerMapMps[i])
        }
        val oldIds = viewerMaps.map { it.id }
        viewerMaps = list
        // 轨迹条数：每次刷新重算（目录里只有几个文件）。
        viewerTraceCounts = list.associate { m ->
            m.id to (sessionDir?.let { TrajectoryStore.tracesFor(it, m.id).size } ?: 0)
        }
        if (oldIds != list.map { it.id }) {
            Log.i(TAG, "[VIEW] atlas 图列表：" + list.joinToString { "#${it.id}(${it.kfs}KF)" })
        }
    }

    private fun preferredViewerMapId(): Long {
        val traj = SlamBridge.nativeTrajectoryMapId()
        return viewerMaps.firstOrNull { it.id == traj }?.id
            ?: viewerMaps.lastOrNull()?.id
            ?: -1L
    }

    private fun selectViewerMap(id: Long) {
        if (!viewerOpen) return
        viewerMapId = id
        loadViewerCloud(id)
        loadViewerTraces(id)
        viewerMapView.setHistory(viewerTraces)
        // 路标点也先刷一遍：下面的 fit 要用新图的点算 bbox。
        refreshWaypoints()
        applyViewerSlice()
        updateViewerTitle(SlamBridge.nativeTrajectoryMapId())
        // 刚切过去可能还没布局完（width=0），等一拍再 fit。
        viewerMapView.post {
            if (viewerOpen && viewerMapId == id) fitViewer()
        }
    }

    private fun loadViewerCloud(id: Long) {
        val t0 = SystemClock.elapsedRealtime()
        val n = SlamBridge.nativeMapCloud(id, viewerCloudBuf)
        viewerCloudN = n
        viewerCloud = if (n > 0) viewerCloudBuf.copyOf(n * 3) else FloatArray(0)
        viewerMedianZ = medianZ(viewerCloud, viewerCloudN)
        val dt = SystemClock.elapsedRealtime() - t0
        Log.i(TAG, "[VIEW] 图 #$id 点云 $n 点，${dt}ms")
        if (dt > 20) Log.w(TAG, "[VIEW] 点云枚举花了 ${dt}ms（主线程）")
    }

    /** 读取指定图自己的轨迹；map-<id>/ 中的文件已在该图坐标系中，使用单位变换。 */
    private fun loadViewerTraces(id: Long) {
        val session = sessionDir
        val list = ArrayList<FloatArray>()
        if (session != null) {
            for (f in TrajectoryStore.tracesFor(session, id)) {
                val n = SlamBridge.nativeReadTrajFile(
                    f.absolutePath,
                    1f, 0f, 0f, 0f, 1f,
                    0f, 0f, 0f,
                    viewerTraceBuf)
                if (n > 0) list += viewerTraceBuf.copyOf(n * 3)
            }
        }
        viewerTraces = list
    }

    private fun medianZ(pts: FloatArray, n: Int): Float {
        if (n <= 0) return 0f
        val z = FloatArray(n)
        for (i in 0 until n) z[i] = pts[i * 3 + 2]
        z.sort()
        return z[n / 2]
    }

    /** 缩放以装下选中图的点云、轨迹和路径点，必要时包含当前位置。 */
    private fun fitViewer() {
        if (!viewerOpen) return
        var minX = Float.MAX_VALUE
        var minY = Float.MAX_VALUE
        var maxX = -Float.MAX_VALUE
        var maxY = -Float.MAX_VALUE
        var any = false

        fun add(x: Float, y: Float) {
            if (x < minX) minX = x
            if (x > maxX) maxX = x
            if (y < minY) minY = y
            if (y > maxY) maxY = y
            any = true
        }

        val c = viewerCloud
        for (i in 0 until viewerCloudN) add(c[3 * i], c[3 * i + 1])
        for (t in viewerTraces) {
            for (i in 0 until t.size / 3) add(t[3 * i], t[3 * i + 1])
        }
        for (i in 0 until wpN) {
            val flags = wpMeta[i * 4 + 3]
            if ((flags and kFlagWpValid) == 0L) continue
            if (wpMeta[i * 4 + 1] != viewerMapId) continue
            add(wpPos[i * 3], wpPos[i * 3 + 1])
        }
        val trajId = SlamBridge.nativeTrajectoryMapId()
        if (viewerMapId == trajId && slamRunning && poseValid) add(poseBuf[12], poseBuf[13])

        if (any) viewerMapView.fitToBounds(minX, minY, maxX, maxY)
        else viewerMapView.fitToBounds(-5f, -5f, 5f, 5f)
    }

    private fun showMapListDialog() {
        if (restoring) {
            viewerDialog("正在载入地图…")
            return
        }
        refreshViewerList()
        if (viewerMaps.isEmpty()) {
            viewerDialog("atlas 里还没有图")
            return
        }

        val trajId = SlamBridge.nativeTrajectoryMapId()
        val labels = viewerMaps.map { m ->
            val tag = when {
                m.id == trajId && slamRunning -> " · 当前"
                m.id == trajId -> " · 上次"
                else -> ""
            }
            "图 #${m.id} · ${m.kfs} KF · ${m.mps} 点 · " +
                "${viewerTraceCounts[m.id] ?: 0} 条轨迹$tag"
        }.toTypedArray()

        MaterialAlertDialogBuilder(this)
            .setTitle("查看哪张图")
            .setItems(labels) { _, which -> selectViewerMap(viewerMaps[which].id) }
            .setNegativeButton("取消", null)
            .show()
    }

    private fun jumpToCurrentMap() {
        if (restoring) {
            viewerDialog("正在载入地图…")
            return
        }
        val id = SlamBridge.nativeTrajectoryMapId()
        if (id < 0) {
            viewerDialog("现在没有当前图")
            return
        }
        refreshViewerList()
        if (viewerMaps.none { it.id == id }) {
            viewerDialog("当前图 #$id 还没有关键帧")
            return
        }
        selectViewerMap(id)
    }

    /** 刷新列表、点云和轨迹，但保持当前平移和缩放。 */
    private fun reloadViewerMap() {
        if (!viewerOpen || viewerMapId < 0) return
        if (restoring) {
            viewerDialog("正在载入地图…")
            return
        }
        refreshViewerList()
        if (viewerMaps.none { it.id == viewerMapId }) {
            val id = preferredViewerMapId()
            if (id >= 0) selectViewerMap(id)
            return
        }
        loadViewerCloud(viewerMapId)
        loadViewerTraces(viewerMapId)
        viewerMapView.setHistory(viewerTraces)
        applyViewerSlice()
        updateViewerTitle(SlamBridge.nativeTrajectoryMapId())
    }

    private fun updateViewerTitle(trajId: Long) {
        val m = viewerMaps.firstOrNull { it.id == viewerMapId }
        if (m == null) {
            tvViewerMap.text = "地图"
            return
        }
        val tag = when {
            m.id == trajId && slamRunning -> " · 当前位置"
            m.id == trajId -> " · 上次位置"
            trajId >= 0 -> " · 当前位姿在图 #$trajId"
            else -> ""
        }
        tvViewerMap.text = "图 #${m.id} · ${m.kfs} KF · ${m.mps} 点 · " +
            "${viewerTraceCounts[m.id] ?: 0} 条轨迹$tag"
    }

    private fun updateViewerSliceLabel() {
        btnViewerSlice.text = if (viewerSliceMedian) "切片:中位数" else "切片:高度"
    }

    /** 打开、切图或切换模式时，按当前模式同步切片中心。 */
    private fun applyViewerSlice() {
        viewerMapView.setSliceCenter(
            if (viewerSliceMedian) viewerMedianZ else mapView.sliceCenter())
    }

    private fun setViewerHint(msg: String) {
        viewerMapView.submit(MapView.Snapshot(
            FloatArray(0), 0, 0f, 0f, 0f, false, msg, 0f, 0f, FloatArray(0), 0))
    }

    private fun viewerDialog(msg: String) {
        MaterialAlertDialogBuilder(this).setMessage(msg).setPositiveButton("好", null).show()
    }

    // ---------------- 小地图的开关和切片厚度 ----------------

    /**
     * 显示或隐藏小地图。
     * 隐藏时同时停止 C++ 侧的点云快照和 Java 侧的轨迹读取。
     * 轨迹记录本身由 ORB-SLAM3 维护，不受影响。
     */
    private fun applyMapVisible(on: Boolean) {
        mapWrap.visibility = if (on) View.VISIBLE else View.GONE
        // 别让人去调一个看不见的东西。
        sbMapRange.isEnabled = on
        sbMapRange.alpha = if (on) 1f else 0.4f
        tvMapRange.alpha = if (on) 1f else 0.4f
        SlamBridge.nativeSetMapCloudEnabled(on)
        // 隐藏时 refreshHistory 会清掉历史层，重新打开需要重读。
        histDirty = true
        refreshHistory()
        refreshMap()
    }

    /** 对齐到滑块的步长；Slider 拒绝非对齐的 value。 */
    private fun snapRange(m: Float): Float {
        val i = ((m - MapView.SLICE_MIN) / kRangeStep).roundToInt()
        return (MapView.SLICE_MIN + i * kRangeStep).coerceIn(MapView.SLICE_MIN, MapView.SLICE_MAX)
    }

    private fun applyMapRange(m: Float) {
        mapView.setSliceRange(m)
        tvMapRange.text = "地图高度范围　%.2f m".format(m)
    }

    // ---------------- 缩放键 ----------------

    /**
     * 缩放键：单击走一档，按住连续走，松手或取消时停止。
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
            true                // 消费事件，避免长按后再次触发点击
        }
        // 返回 false，保留 View 默认的点击和长按处理。
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

    /** 保存缩放档位，仅在档位变化时写入。 */
    private fun saveZoom() {
        val i = mapView.zoomIndex()
        if (i == zoomSaved) return
        zoomSaved = i
        pref.edit().putInt(PREF_MAP_ZOOM, i).apply()
    }

    /**
     * 跟踪状态文字。
     * 惯性初始化需要运动，静止时无法完成，提示中需要说明这一点。
     */
    /** 取主题色；动态取色之后静态资源不再等于实际颜色，必须走 attr。 */
    private fun materialColor(attr: Int): Int = MaterialColors.getColor(btnSlam, attr)

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
            uiMessage = "相机权限被拒绝。\n请到 设置 → 应用 → SlamCapture → 权限 里手动开启。"
            render()
        }
    }

    // ---------------- 文件 ----------------

    /**
     * 词袋路径。文件未打入 APK，需要推送到外部文件目录。
     * 返回 null 表示状态栏已显示提示，调用方不应继续调用 nativeInit。
     */
    private fun resolveVocabPath(): String? {
        val f = File(getExternalFilesDir(null) ?: filesDir, VOCAB_NAME)
        return f.takeIf { it.isFile && it.length() > 0 && it.canRead() }?.absolutePath
            ?: refuse("词袋", f) { "跑一次 tools/push-vocab.sh 把它推上来（145 MB，不进 APK）。" }
    }

    /**
     * yaml 路径。模板来自 assets，只替换相机–IMU 外参和 IMU 频率；
     * **内参不写进文件**，由 [applyIntrinsics] 走 nativeSetIntrinsics 注入。
     */
    private fun resolveSettingsPath(): String? {
        val dst = File(filesDir, SETTINGS_NAME)

        // 外参旋转取决于这台机器的传感器方向，不能沿用模板里那台的值。
        val info = cam.cameraInfo()
        if (info == null) {
            return refuse("相机信息", dst) { "打不开相机，算不出相机–IMU 外参。" }
        }
        cameraId = info.first
        sensorOrientation = info.second

        val imuFreq = CalibStore.load(this)?.imuFrequency?.takeIf { it > 0f }
            ?: IMU_FREQ_FALLBACK
        try {
            val template = assets.open(SETTINGS_NAME).bufferedReader().use { it.readText() }
            val text = buildString {
                append(SettingsTemplate.patch(
                    template, SettingsTemplate.tbc(sensorOrientation), imuFreq))
                // 选择旧图时补充路径。System 构造时读取该键，载图只在那一刻生效；
                // 值不带 .osa，上游会自行追加。
                mapFile?.let { append("\nSystem.LoadAtlasFromFile: \"${it.removeSuffix(".osa")}\"\n") }
            }
            dst.writeText(text)
        } catch (e: IOException) {
            Log.e(TAG, "释放 $SETTINGS_NAME 失败", e)
            uiMessage = "打不开 assets/$SETTINGS_NAME：$e"
            render()
            return null
        }
        return dst.takeIf { it.length() > 0 && it.canRead() }?.absolutePath
            ?: refuse("配置", dst) { "assets/$SETTINGS_NAME 是空的，检查打包。" }
    }

    /**
     * 路径检查。路径错误时 ORB-SLAM3 会直接 exit(-1)，应尽量在这里拦住；
     * 缺少必填项的情况只能依靠 logcat 中的日志。
     */
    private fun refuse(what: String, f: File, hint: () -> String): String? {
        Log.e(TAG, "找不到$what：${f.absolutePath}")
        uiMessage = "找不到$what：\n${f.absolutePath}\n\n${hint()}"
        render()
        return null
    }

    /**
     * 一次新建或打开地图对应一个目录：files/maps/<名字>/。
     * 没给名字时用时间戳 —— 不用 mapId，因为后者会随进程和地图变化。
     */
    private fun newSessionDir(name: String? = null): File {
        // getExternalFilesDir 可能返回 null（外部存储没挂载）。这时退到内部存储，
        // 不能把 null 传给 File 构造器，否则路径会落到进程当前目录。
        val maps = File(getExternalFilesDir(null) ?: filesDir, "maps")
        val clean = name?.let { TrajectoryStore.cleanMapName(it) }.orEmpty()
        if (clean.isNotEmpty()) return TrajectoryStore.uniqueMapDir(maps, clean).apply { mkdirs() }
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

    /** 目录名和地图文件名使用的时间戳格式。 */
    private fun stamp(): String = SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(Date())

    /** 返回 rec/<runStamp>/session-M/，M 取第一个未占用的编号。 */
    private fun nextSessionDir(run: File): File {
        var n = 1
        while (File(run, "session-$n").exists()) n++
        return File(run, "session-$n")
    }

    /** 本趟已录制的段：rec/<runStamp>/session-M/。 */
    private fun sessionDirs(run: File?): List<File> =
        run?.listFiles { f -> f.isDirectory && f.name.startsWith("session-") }
            ?.sorted() ?: emptyList()

    /**
     * 本趟轨迹前缀（不含扩展名）：<会话>/traj/traj-<runStamp>。
     * 图保存前 map id 可能被复用，因此先放在 traj/。
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

    /** 读取本趟合并链，每条 10 个 float。 */
    private fun readMerges(): List<FloatArray> {
        val buf = FloatArray(10 * 16)
        val n = SlamBridge.nativeLastMerges(buf)
        return (0 until n).map { i -> buf.copyOfRange(i * 10, i * 10 + 10) }
    }

    // ---------------- 这一趟留不留 ----------------

    /**
     * 询问是否保留本趟数据。
     * 点击外部或返回键都视为保留，只有明确选择「不保存」才丢弃。
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
        MaterialAlertDialogBuilder(this)
            .setTitle("保存这一趟？")
            .setMessage(msg)
            .setPositiveButton("保存") { _, _ -> keepRun(base) }
            .setNegativeButton("不保存") { _, _ -> onDropTrajectory(base) }
            .setOnCancelListener { keepRun(base) }
            .show()
    }

    /**
     * 结束时的第一问：是否保存地图。完成后继续询问轨迹。
     * 先确定地图文件名，随后写出的轨迹元信息才能记录它。
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
        val copyRec = MaterialCheckBox(this).apply {
            text = "另存时连录像一起拷"
            setPadding(56, 0, 56, 0)
        }

        val b = MaterialAlertDialogBuilder(this)
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
        // 这一趟不要了：把 atlas 退回上一次保存的样子。
        // 不退回的话，这趟建的图会一直留在内存里，下一趟的回环检测还能匹配到它。
        val dropRun = {
            Log.i(TAG, "这一趟不存图 → 请求恢复 atlas")
            atlasSwitching = false
            SlamBridge.nativeRequestRestoreMap()
            uiQueueTicker()   // 恢复的进度和结果要靠 ticker 刷
        }

        // 用户到底选没选「存」。三个肯定按钮都要置 true ——
        // 只有「不保存」和直接关掉（返回键 / 点外面）才该丢图。
        var willSave = false
        // 命名弹窗会接着弹，轨迹的问题等它结束再问，免得两个弹窗叠在一起。
        var naming = false
        if (existing != null) {
            b.setPositiveButton("覆盖") { _, _ ->
                willSave = true
                startMapSave(existing.absolutePath.removeSuffix(".osa"))
            }
                .setNeutralButton("另存为") { _, _ ->
                    willSave = true
                    naming = true
                    askMapName(stamp()) { name ->
                        if (name == null) {
                            willSave = false
                            dropRun()
                        } else {
                            saveMapAs(copyRec.isChecked, name)
                        }
                        askKeepRun(trajBase)
                    }
                }
                .setNegativeButton("不保存", null)
        } else {
            b.setPositiveButton("保存") { _, _ ->
                willSave = true
                naming = true
                askMapName(dir.name) { name ->
                    if (name == null) {
                        willSave = false
                        dropRun()
                    } else {
                        // 先改名再存：pendingArchive 拿的是 sessionDir，存完再改就对不上了。
                        val dst = renameSessionDir(dir, name)
                        if (dst == null) uiMessage = "目录改名失败，已按原名保存"
                        val target = dst ?: dir
                        sessionDir = target
                        startMapSave(File(target, target.name).absolutePath)
                    }
                    askKeepRun(trajBase)
                }
            }
                .setNegativeButton("不保存", null)
        }
        val dlg = b.create()
        // 挂在「弹窗消失」上：两个轴互不影响，问完接着问轨迹。
        dlg.setOnDismissListener {
            if (!willSave) dropRun()
            if (!naming) askKeepRun(trajBase)
        }
        dlg.show()
    }

    /**
     * 地图命名弹窗。取消返回 null（= 这次保存作废）；留空则用 [fallback]。
     * 名字里的路径分隔符会被替换，重名由 [TrajectoryStore.uniqueMapDir] 加后缀。
     */
    private fun askMapName(fallback: String, onResult: (String?) -> Unit) {
        textInputDialog("给这张地图起个名字", fallback, onCancel = { onResult(null) }) { raw ->
            onResult(TrajectoryStore.cleanMapName(raw).ifEmpty { fallback })
        }
    }

    /** 把会话目录改名为 name；名字没变就原样返回，改名失败返回 null。 */
    private fun renameSessionDir(dir: File, name: String): File? {
        val clean = TrajectoryStore.cleanMapName(name)
        if (clean.isEmpty() || clean == dir.name) return dir
        val parent = dir.parentFile ?: return null
        val dst = TrajectoryStore.uniqueMapDir(parent, clean)
        if (!dir.renameTo(dst)) {
            Log.e(TAG, "会话目录改名失败：${dir.absolutePath} → ${dst.absolutePath}")
            return null
        }
        Log.i(TAG, "会话目录改名：${dir.name} → ${dst.name}")
        rebaseSessionPaths(dir, dst)
        return dst
    }

    /** 目录改名后，把还指向旧路径的内存句柄换到新目录下。 */
    private fun rebaseSessionPaths(old: File, new: File) {
        val prefix = old.absolutePath + "/"
        fun rebase(f: File?): File? = f?.let {
            if (it.absolutePath.startsWith(prefix))
                File(new, it.absolutePath.removePrefix(prefix))
            else it
        }
        curTrajBase = rebase(curTrajBase)
        curRecDir = rebase(curRecDir)
        lastRunDir = rebase(lastRunDir)
    }

    /** 让 SLAM 线程把地图保存为 path + ".osa"（path 不含扩展名）。 */
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
    private fun saveMapAs(copyRecordings: Boolean, name: String) {
        val src = sessionDir ?: return
        val dst = newSessionDir(name)
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

        // 本趟文件路径已改变，指针需要同步更新。
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
     * 丢弃本趟轨迹。录制过会话时再次确认是否保留录像。
     */
    private fun onDropTrajectory(base: File) {
        val segs = sessionDirs(curRecDir).size
        if (segs == 0) {
            discardAll(base)
            return
        }
        MaterialAlertDialogBuilder(this)
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
     * 返回本趟轨迹的实际位置，可能仍在 traj/，也可能已归档到 map-<id>/。
     * 存图可能早于是否保留的选择完成，因此需要同时检查两处。
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

    /** 只删除本趟轨迹和元信息，保留录像。 */
    private fun dropTrajectoryOnly(base: File) {
        runTrajFile(base, ".txt").delete()
        runTrajFile(base, ".json").delete()
        lastRunDir = curRecDir
        Log.i(TAG, "丢掉轨迹、保留会话：${base.absolutePath}")
        render()
    }

    /**
     * 丢弃整趟数据：删除轨迹和录像。
     * 不能删除整个会话目录，其中可能包含其他趟的数据。
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
     * 写入 traj-<stamp>.json 元信息。
     * 轨迹只包含跟踪 OK 的帧，地图重置后只保留最后一段。
     * merges 记录坐标系的来源，归档完成后无法再从目录结构推断。
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

    /** 合并链 JSON：[[survivor, absorbed, s, qx, qy, qz, qw, tx, ty, tz], ...]。 */
    private fun mergesJson(): String =
        runMerges.joinToString(", ", "[", "]") { r ->
            "[" + r[0].toLong() + ", " + r[1].toLong() + ", " +
                (2..9).joinToString(", ") { i -> r[i].toString() } + "]"
        }

    /**
     * 存图成功后把本趟轨迹归档到 map-<id>/，并按合并链统一磁盘坐标。
     *
     * 只在保存中变为完成时调用一次；必须按链逐条处理：
     * 先变换幸存目录中的现有文件，再将被吸收目录原样移入。
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

            // ① 按 T_on 变换已有轨迹，先全部写 .tmp，成功后统一改名。
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

            // ② 被吸收目录原样移入，清空后删除。
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

        // ③ 本趟轨迹已在最终坐标系中，直接移入，不做变换。
        if (id < 0) return   // 没有图：松文件留在 traj/
        val base = p.base ?: return
        val dst = TrajectoryStore.mapDir(session, id).apply { mkdirs() }
        moveInto(File("${base}.txt"), dst)
        moveInto(File("${base}.json"), dst)
        lastRunDir = dst
    }

    /** 把 [f] 移入 [dst]，重名时加 -2；失败返回 false，调用方不应删除源目录。 */
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
     * 写入 <会话>/session.json，作为会话汇总，只在建目录时写一次。
     * 名字不使用 traj-*.json，以免与趟级元信息混淆。
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
