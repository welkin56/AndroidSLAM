// 构造并持有 `ORB_SLAM3::System`。Kotlin 侧只问状态，不碰任何句柄。
//
// 为什么只有自由函数、没有句柄：§6 那条约定 ——「图」的概念完全不跨 JNI。
// Kotlin 侧看不到 Map / Atlas / referenceMap，也没有任何东西要传回来。
#pragma once

#include <string>

namespace slam {

// 用 int 报给 Kotlin。四态是必需的：「失败」得能显示出来，
// 否则 UI 就是「点了一下，一直转圈」。
enum class State {
    kIdle = 0,     // 还没调过 init
    kLoading = 1,  // 词袋在加载（十几到几十秒）
    kReady = 2,    // System 构造好了
    kFailed = 3,   // 构造抛异常了（⚠️ 走 exit(-1) 的那种根本走不到这里）
};

// 只生效一次，重复调用直接忽略。**同步只做「存路径 + 置 kLoading」，立刻返回** ——
// 真正的构造在别的线程上跑，否则「启动」按钮会被卡死十几到几十秒。
void init(const std::string& vocabPath, const std::string& settingsPath,
          const std::string& loadFromFile);

State state();

// 「已经加载了多久」，毫秒；不在加载中时返回 0。
//
// 为什么要由 C++ 报而不是 Kotlin 自己掐表：**构造是进程级的事实，掐表是 Activity 级的状态**。
// Activity 重建（比如从入口页再进来一次）时 Kotlin 那边的计时会归零，而 System 还在造 ——
// 界面就会显示成「刚点下」然后一直转圈。谁干的活谁报时间。
long long elapsedMs();

}  // namespace slam
