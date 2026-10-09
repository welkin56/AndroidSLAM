# 2026-10-08 修掉上游 ORB-SLAM3 里一个「必然崩」的分支

一句话：`Optimizer.cc` 里那句「顶点缺失」的报错信息，**恰恰在它要诊断的那种情况下会崩**。
删掉那句，**计算路径一个字没变**。

## 动机

真机测距时 App 闪退。tombstone：

```
Fatal signal 11 (SIGSEGV), code 1 (SEGV_MAPERR), fault addr 0x8
Cause: null pointer dereference
  #00  Optimizer::InertialOptimization(Map*, Matrix3d&, double&)
  #01  LocalMapping::ScaleRefinement()
  #02  LocalMapping::Run()
```

**整条栈都在 ORB-SLAM3 的 LocalMapping 线程里**，而当时的测距代码只跑在跟踪线程、
而且只读地图点（`isBad()` / `GetWorldPos()`，各自带锁）。所以要先把「是不是我们弄的」排掉。

**排掉的过程**（两条都查了源码）：

1. **我缓存了一个 `MapPoint*`**（锁定用），它会不会变野指针？——
   不会。`Map::clear()` 和 `Map::~Map()` 里那两行 `delete` **都被上游注释掉了**，
   `Atlas` 只 `delete Map` 对象、不释放 MapPoint。所以那些指针活到进程结束。
2. **测距会不会把上游的指针弄空？** —— 不会，它只读。

## 改动

- **`app/src/main/cpp/thirdparty/orb_slam3/src/Optimizer.cc`** —— `InertialOptimization(Map*, Matrix3d&, double&)`
  里那句拼错误串的 `Verbose::PrintMess("Error" + to_string(VP1->id()) + ...)` 换成一个不碰指针的常量串。
- **`app/src/main/cpp/thirdparty/README.md`** —— 改动表加**第 10 条**，
  并专门注明**它和前面 9 条性质不同**（前 9 条是「在安卓上编得过」的迁移，这条是上游 bug）。

## 决策

### 一、定位方法：`addr2line` 对着**没 strip 的** `.so`

崩溃时那台手机上跑的那个 `.so`，构建目录里有一份**带符号**的：

```bash
llvm-addr2line -C -f -i -e \
  app/build/intermediates/cxx/Debug/<hash>/obj/arm64-v8a/liborb_slam3.so  0x3b2ca4
```

输出：

```
g2o::HyperGraph::Vertex::id() const
    .../thirdparty/g2o/g2o/core/hyper_graph.h:101
ORB_SLAM3::Optimizer::InertialOptimization(...)
    .../thirdparty/orb_slam3/src/Optimizer.cc:3464
```

**`fault addr 0x8` + `id()` 正好是读对象偏移 8 的成员** —— 对上了。

> ⚠️ **这条经验值得单独记**：tombstone 里的 `pc` 地址是 `.so` 内的偏移，
> 而 AGP 的 `intermediates/cxx/Debug/<hash>/obj/<abi>/` 下那份**没被 strip**。
> 不用符号服务器、不用 ndk-stack，一条 `addr2line` 就够。
> （`build/intermediates/**/stripped_native_libs/` 下那份是被 strip 过的，别用它。）

### 二、根因：**判空之后又解引用**

```cpp
if(!VP1 || !VV1 || !VG || !VA || !VP2 || !VV2 || !VGDir || !VS)
{
    Verbose::PrintMess("Error" + to_string(VP1->id()) + ", " + to_string(VV1->id()) + ...,
                       Verbose::VERBOSITY_NORMAL);
    continue;
}
```

`if` 刚刚判定「其中至少有一个是空指针」，紧接着那句「报错」把**全部八个**都解引用了。

**更讽刺的是它还带 `VERBOSITY_NORMAL`，而 `Verbose::th` 在 `System` 构造函数末尾就被设成了
`QUIET`（`System.cc:239`）—— 所以这句永远不打印，只有崩的那一半在执行。**

### 三、为什么这不违反 `thirdparty/README.md` 那条「没动算法」

那张改动表最后写着「**全都是为了在安卓上编得过，没有一处动到算法**」。
而这条**不是编译迁移**，是改代码。之所以还是按「没动算法」算：

- 那个分支**本来就是 `continue`** —— 「跳过这条边」是上游的原意；
- 删掉的只是一句**永远不会被打印、且必然崩溃**的字符串拼接。

**计算路径一个字都没变，所以 PC 对照（`mono_inertial_euroc`）仍然成立。**

⚠️ 但它**没有解决「谁把顶点弄空了」** —— 那是 `LocalMapping` 在初始化 / 重置期间的老问题。
只是上游本来就打算跳过那条边，所以不崩是对的。

### 四、什么时候会被触发（顺带查清的）

`LocalMapping::Run()` 里：

```cpp
if ((KeyFramesInMap() <= 200) &&
    (mTinit 落在 25/35/45/55/65/75 秒的 ±0.25 秒窗口里))
    ScaleRefinement();          // → InertialOptimization
```

**⚠️ `mTinit` 从不重置**（只在 `LocalMapping` 构造时归零），只在 `bInitializing` 期间累加。
**而我们的 App 故意不销毁 `System`**（为了不重付 38 秒词袋），所以它**跨「启动 / 结束」继续涨** ——
上游假设的是**一次连续会话**。这就是为什么跑了 35 分钟才撞上。

### 五、**否决**：连 `SLAMAPP/ORB_SLAM3/` 一起改

`thirdparty/README.md` 明说「**只改这里这份**，`SLAMAPP/ORB_SLAM3/` 只当上游参照」。
**没改。** 代价是 PC 上如果也撞上同一条分支，会崩在同一行 —— 已写进 README 第 10 条。

## 验证

**1. 编译通过**

```
$ ./gradlew assembleDebug
BUILD SUCCESSFUL in 45s
```

**2. 那句自崩的字符串真的从二进制里消失了**

```
$ grep -ac 'Error" + to_string' liborb_slam3.so     → 0
$ grep -ac 'missing vertex, skip edge' liborb_slam3.so → 1
```

（`strings` 默认只认 ASCII，中文格式串会被切断，所以直接对二进制 `grep -a`。）

**3. 真机跑了一趟，没有再崩**

```
$ adb logcat -b crash -d
（空）
```

⚠️ **一趟不算数** —— 触发条件是 0.5 秒宽的窗口，本来就是撞运气。

**4. 未验证**

- 崩溃时**具体是哪一个顶点为空**没有查（只知道「至少一个」）。
  `maxKFid` 是 `pMap->GetMaxKFid()`，而顶点 id 是 `2*(maxKFid+1)+mnId` 之类算出来的 ——
  `KeyFrame::mnId` 和 `maxKFid` 的关系在**地图被重置过之后**会错位，**这是可疑点，但没查**。
- 那条分支多久走一次、走的时候丢了多少条边 —— **没有埋点，不知道**。

## 遗留

- **「谁把顶点弄空了」没解决** —— 见「决策」三。
- **`mTinit` 跨会话累加**这件事**没有处理**。它让 `ScaleRefinement` 在**地图状态未知**的时刻触发。
  真要处理，得让 `System` 每次「启动」重置初始化状态 —— 但那要动上游更多的地方。
- **PC 那份没改**（见「决策」五）。

## 回滚

`git revert` 这一个提交即可。手动的话：
`Optimizer.cc` 把 `Verbose::PrintMess(...)` 那句换回原来那个拼 `->id()` 的长串；
`thirdparty/README.md` 删掉第 10 条那一段。
