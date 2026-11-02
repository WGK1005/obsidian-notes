主页面 [[感知建图]]
```
https://www.redblobgames.com/pathfinding/a-star/introduction.html
```
以上是一个非常不错的网站，内含A*和DIJKSTRA算法的代码和动画讲解
# 一总体概念
- Dijkstra 算法： 不知道目标在哪个方向更值得优先探索时，按照从起点出发的累计代价，由近及远搜索。（代价地图，注重找一条代价低的路，物理上看起来是长一点但绝对是代价最低的）
    
- A 算法：* 利用对目标的估计距离，引导搜索优先朝着更有希望的方向进行。（预估价值最大的图，并不在意代价，是物理意义上的最短）
路径规划可以抽象为一个图搜索问题。

- 节点（Node）： 机器人可以到达的状态，例如地图上的一个栅格。
- 边（Edge）： 两个节点之间允许进行的移动。
- 边权（Cost）： 移动所需的距离、时间、能耗或风险。
- 起点（Start）： 机器人当前所在的位置。
- 终点（Goal）： 机器人希望到达的位置。
- 路径（Path）： 从起点到终点的一系列节点。
# 二 Dijkstra 算法：从起点向外扩散
## 初始化
它的核心思想是：
> 每次从尚未确定最短距离的节点中，选择当前已知起点距离最小的节点，再用它更新相邻节点的距离。

Dijkstra 算法本质上是**加权图上的广度优先搜索（BFS）**。它维护一个从起点 $s$ 到当前节点 $n$ 的实际累计移动代价 $g(n)$：

- 起点代价初始化：$g(s) = 0$
- 其余节点初始代价：$g(v) = \infty$

算法通过边松弛（Relaxation）逐步逼近各节点的最短距离：

$$g(v) = \min\big(g(v),\, g(u) + c(u, v)\big)$$

其中 $c(u, v)$ 为节点 $u$ 到邻接节点 $v$ 的移动边权。
## 算法执行流程
- **初始化**：
    
    - 创建优先级队列 `OpenSet`（最小堆），存储待考察节点，按 $g(n)$ 升序排序。
        
    - 创建 `ClosedSet`（集合或布尔访问数组），记录已确定最短路径的节点。
        
    - 将起点加入 `OpenSet`。
        
- **循环扩展**：
    
    - 从 `OpenSet` 中取出 $g(u)$ 最小的节点 $u$，加入 `ClosedSet`。
        
    - 若 $u$ 即为目标节点，则回溯父指针生成路径并终止。
        
    - 遍历 $u$ 的所有未在 `ClosedSet` 中的邻居节点 $v$：
        
        - 计算临时累计代价：$g_{temp} = g(u) + c(u, v)$。
            
        - 若 $g_{temp} < g(v)$，则更新 $g(v) = g_{temp}$，记录 $v$ 的父节点为 $u$，并将 $v$ 压入或更新于 `OpenSet` 中。
            
- **重复**直至找到目标点或 `OpenSet` 为空（无可行解）。
> [!info] 知识点
> Dijkstra 适用于边权非负的图。

>[!tip] 感想
>这个其实不太适合RC感知，其实并不需要太考虑代价问题，因为路是平的，而且太吃内存了
# A*算法(更常用)
>A*（A-Star属于**启发式搜索算法（Informed Search）**。它将 Dijkstra 算法的“已知代价”与贪婪最佳优先搜索（Greedy Best-First Search）的“预估代价”结合起来。

## 核心评价函数

A* 算法的核心在于为每个节点 $n$ 定义了一个综合评估函数：

$$f(n) = g(n) + h(n)$$

- **$g(n)$（实际代价）**：从起点到当前节点 $n$ 的实际累计代价（同 Dijkstra）。
    
- **$h(n)$（启发代价，Heuristic）**：从节点 $n$ 到目标节点 $goal$ 的预估代价。
    
- **$f(n)$（总估算代价）**：经过节点 $n$ 的整条路径的预计最小总代价。`OpenSet` 始终弹出 $f(n)$ 最小的节点进行扩展。
    

## 启发函数的性质（最优性保证）

启发函数 $h(n)$ 的设计直接决定了算法的最优性与执行效率：

1. **可采纳性（Admissibility）**： 对于任意节点 $n$，均有 $h(n) \le h^*(n)$（其中 $h^*(n)$ 为 $n$ 到目标的真实物理最短代价），即启发函数从不高估实际距离。
    
    - **性质**：只要 $h(n)$ 满足可采纳性，A* 算法一定能找到理论全局最优解。
        
    - **特例**：若令 $h(n) = 0$，则 $f(n) = g(n)$，A* 完全退化为 Dijkstra 算法。
        
2. **一致性 / 单调性（Consistency / Monotonicity）**： 对于任意相邻节点 $n$ 与 $n'$，均满足三角不等式：
    
    $$h(n) \le c(n, n') + h(n')$$
    
    且 $h(goal) = 0$。
    
    - **性质**：一致性是比可采纳性更强的条件。若 $h(n)$ 满足一致性，则任一节点一旦从 `OpenSet` 弹出并放入 `ClosedSet`，其 $g(n)$ 必然已经是最优解，**永远不需要对已闭合的节点进行重新开放（Re-open）**，大幅减少计算开销。
        

### 常见启发距离度量

在二维空间或网格地图中，根据移动自由度的不同选择对应的启发函数：

- **曼哈顿距离（Manhattan Distance）**：仅允许 4 邻域上下左右移动（网格世界）。
>[!tip] 翻译
>就是只能直来直去，没有斜线，是更常用的选择

    
    $$h(n) = D \cdot (\vert{}x_n - x_{goal}\vert{} + \vert{}y_n - y_{goal}\vert{})$$
    
- **切比雪夫 / 对角线距离（Diagonal / Chebyshev Distance）**：允许 8 邻域移动且对角线移动代价与直线相同。
    
- **欧几里得距离（Euclidean Distance）**：机器人可在连续平面任意角度直线移动。
    >就是能斜线，但是算起来太麻烦，不利于动态性规划
    $$h(n) = D \cdot \sqrt{(x_n - x_{goal})^2 + (y_n - y_{goal})^2}$$
## 更详细的解读
A* 评估函数为 $f(n) = g(n) + h(n)$。在二维网格几何规划中，启发函数 $h(n)$ 需满足两个层级的约束：

#### 1. 可采纳性（Admissibility）

$$\forall n \in V,\quad 0 \le h(n) \le h^*(n)$$

其中 $h^*(n)$ 为从 $n$ 到目标点 $g$ 的物理真实最优代价。该条件保证算法终止时返回的解必定全局最优。

#### 2. 一致性 / 单调性（Consistency / Monotonicity）

$$\forall (n, n') \in E,\quad h(n) \le c(n, n') + h(n') \quad \text{且} \quad h(goal) = 0$$

即启发函数满足**三角不等式**。

> **单调性推论**：若 $h(n)$ 满足一致性，则沿任意扩展路径，$f(n)$ 的值单调非递减。

- **推导**： 设 $n'$ 是由节点 $n$ 扩展出的后继节点，根据代价累加关系有 $g(n') = g(n) + c(n, n')$：
    
    $$\begin{aligned} f(n') - f(n) &= \big(g(n') + h(n')\big) - \big(g(n) + h(n)\big) \\ &= g(n) + c(n, n') + h(n') - g(n) - h(n) \\ &= c(n, n') + h(n') - h(n) \end{aligned}$$
    
    由一致性定义 $h(n) \le c(n, n') + h(n')$，直接得出：
    
    $$f(n') - f(n) \ge 0 \implies f(n') \ge f(n)$$
    

**工程收益**：当 $h(n)$ 满足一致性时，任何节点一旦首次从 `OpenSet` 弹出并放入 `ClosedSet`，其 $g(n)$ 就已经收敛到理论最小值。**算法不需要对已闭合节点进行重新展开（Re-opening）**，状态机时间复杂度稳定维持在严格的 $O(\vert{}E\vert{} \log \vert{}V\vert{})$，避免竞赛算力抖动。
# 代码实现
```
import heapq
import math

# 8个移动方向及对应移动代价: (dr, dc, cost)
MOTIONS = [
    (-1,  0, 1.0),
    ( 1,  0, 1.0),
    ( 0, -1, 1.0),
    ( 0,  1, 1.0),
    (-1, -1, math.sqrt(2)),
    (-1,  1, math.sqrt(2)),
    ( 1, -1, math.sqrt(2)),
    ( 1,  1, math.sqrt(2)),
]


def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def dijkstra(grid, start, goal):
    """
    Dijkstra 算法 (grid 中 0 表示可行走，1 表示障碍物)
    :param grid: List[List[int]] 二维网格
    :param start: Tuple[int, int] (row, col)
    :param goal: Tuple[int, int] (row, col)
    :return: 路径节点列表 或 None
    """
    rows, cols = len(grid), len(grid[0])
    
    # 优先队列: (g_score, (r, c))
    pq = []
    heapq.heappush(pq, (0.0, start))
    
    g_score = {start: 0.0}
    came_from = {}
    visited = set()

    while pq:
        current_g, current = heapq.heappop(pq)

        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            return reconstruct_path(came_from, current)

        r, c = current
        for dr, dc, cost in MOTIONS:
            nr, nc = r + dr, c + dc
            neighbor = (nr, nc)

            # 越界检查与障碍物检查
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if grid[nr][nc] == 1 or neighbor in visited:
                continue

            new_g = current_g + cost
            if new_g < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = new_g
                came_from[neighbor] = current
                heapq.heappush(pq, (new_g, neighbor))

    return None


def a_star(grid, start, goal):
    """
    A* 算法 (grid 中 0 表示可行走，1 表示障碍物)
    :param grid: List[List[int]] 二维网格
    :param start: Tuple[int, int] (row, col)
    :param goal: Tuple[int, int] (row, col)
    :return: 路径节点列表 或 None
    """
    rows, cols = len(grid), len(grid[0])

    def heuristic(p1, p2):
        # 欧几里得启发式距离
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    # 优先队列: (f_score, (r, c))
    pq = []
    heapq.heappush(pq, (heuristic(start, goal), start))

    g_score = {start: 0.0}
    came_from = {}
    visited = set()

    while pq:
        current_f, current = heapq.heappop(pq)

        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            return reconstruct_path(came_from, current)

        r, c = current
        current_g = g_score[current]

        for dr, dc, cost in MOTIONS:
            nr, nc = r + dr, c + dc
            neighbor = (nr, nc)

            # 越界检查与障碍物检查
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if grid[nr][nc] == 1 or neighbor in visited:
                continue

            new_g = current_g + cost
            if new_g < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = new_g
                came_from[neighbor] = current
                f_score = new_g + heuristic(neighbor, goal)
                heapq.heappush(pq, (f_score, neighbor))

    return None


# ================== 测试运行 ==================
if __name__ == "__main__":
    # 0 为平地，1 为障碍
    map_grid = [
        [0, 0, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 1, 0],
        [0, 1, 0, 0, 0],
        [0, 0, 0, 0, 0]
    ]
    start_pos = (0, 0)
    goal_pos = (4, 4)

    path_dijkstra = dijkstra(map_grid, start_pos, goal_pos)
    path_astar = a_star(map_grid, start_pos, goal_pos)

    print("Dijkstra 路径:", path_dijkstra)
    print("A* 路径:      ", path_astar)
```
