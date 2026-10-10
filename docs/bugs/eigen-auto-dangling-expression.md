# `auto` 接住 Eigen 的表达式，函数返回后它指向已销毁的临时对象

**症状：不崩、不报错、编译通过 —— 但读到的数是垃圾，而且是一大片静默失败。**

2026-10-10 在这上面卡了三轮真机，**一个测距读数都出不来**，而我一直以为「是参数没调对」。

## 长什么样

```cpp
auto pcOf = [&](const float pw[3]) {
    return (Tcw * Eigen::Vector3f(pw[0], pw[1], pw[2]).homogeneous()).head<3>();
};   //  ↑ auto 推出的是 Eigen::Block 表达式，不是一个 Vector3f
```

调用方：

```cpp
const Eigen::Vector3f q = pcOf(s.pw);   // ← 到这里才求值，读的是已释放的内存
```

**一眼看不出问题** —— 写法在别的地方（`Eigen::Vector3f q = Tcw * v;`）完全正常。
**问题出在「把表达式当返回值传出去」。**

## 为什么

**Eigen 的核心设计是表达式模板**：算术运算**不立刻算**，而是构造一个描述运算的轻量对象，
真正的计算推迟到「赋值给一个具体类型」那一刻。

所以上面那一串里**没有任何一个具体对象**，全是指向临时的表达式：

| 子表达式 | 是什么 | 引用谁 |
|---|---|---|
| `Eigen::Vector3f(pw[0],pw[1],pw[2])` | **具体的临时对象** | — |
| `.homogeneous()` | `CwiseUnaryOp` 表达式 | ↑ 那个临时 |
| `Tcw * …` | `Product` 表达式 | Tcw + 上面那个 |
| `.head<3>()` | `Block` 表达式 | 上面那个 Product |

`return` 时：

1. `auto` 把返回类型推成 **`Block` 表达式**
2. **return 语句结束时，那个 `Eigen::Vector3f` 临时对象被销毁**
3. **返回出去的 `Block` 仍然指着那块内存**（悬垂）

调用方那句 `const Eigen::Vector3f q = pcOf(...)` 才触发求值 —— **读的是垃圾**。

## 为什么特别难查

1. **不崩、不报错、编译通过。** 它是 UB，不是异常。
2. **症状是「静默的成片失败」**，不是一处崩溃：
   垃圾 `q.z()` 大概率是负数或极大值 ⟹ 被下游那些合理的守卫筛掉
   （`if (q.z() <= kMinZ) continue;`、`if (角度 > 阈值) continue;`）⟹ **每个候选都被滤掉，但不报任何错**。
3. ⚠️ **它和「参数没调对」长得一模一样。** 表现出来都是「什么都不出来」，
   于是我连着改了两轮常量 —— 都白改。
4. **同一个 lambda 里的其他代码看起来是对的**（`Tcw` 捕获正确、公式正确）。

## 怎么修

**显式写返回类型**，强制在临时还活着的时候求值：

```cpp
auto pcOf = [&](const float pw[3]) -> Eigen::Vector3f {
    return (Tcw * Eigen::Vector3f(pw[0], pw[1], pw[2]).homogeneous()).head<3>();
};
```

等价写法：`return (...).eval();`，或者先存进一个具名变量再返回。

## 怎么认出来 / 怎么防

**判据：任何「返回类型由 `auto` 推导、而 `return` 的是一个 Eigen **表达式**」的函数或 lambda。**

- **返回类型里带 Eigen 的类型 ⟹ 显式写出来。** 别用 `auto`。
- ⚠️ **不要只看「`auto` 后面是不是 Eigen 类型」** —— 关键是**返回的那个东西是不是表达式**
  （`Block` / `Product` / `CwiseUnaryOp` …它们长得不像 Eigen 类型）。
- **返回 `bool` / `int` / 标量的 lambda 是安全的**（那次同时存在的另外两个 lambda 都没事）。

## 什么时候还会撞上

- **任何「取一小块再返回」的场景**：`.head<>()` / `.block<>()` / `.segment()` / `.row()` / `.col()`
- **任何「算完再返回」的场景**：`A * B` / `A + B` / `.cwiseAbs()` / `A.transpose()`
- **返回值里带着对入参的引用**：`const auto& m = Tcw.matrix();`（`matrix()` 按值返回时同样悬垂）
- **存进容器**：`std::vector<Eigen::Matrix3f> v; v.push_back(A * B);` —— 这个反而**安全**，
  因为 `push_back` 的形参类型是具体的 `Matrix3f`，会在临时还活着时求值 ✓

> **一句话：Eigen 的类型出现在「接口」上时，一律显式写出来。**

## 相关

- 真机上的完整排查过程：[changes/2026-10-10-support-ring-and-texture-density.md](../changes/2026-10-10-support-ring-and-texture-density.md)
- 另一类同样「检测到 → 打印 → 照样用」的上游坑：[changes/2026-10-08-upstream-optimizer-null-deref.md](../changes/2026-10-08-upstream-optimizer-null-deref.md)
