# <img src="./PIC/logo.png" height="40" /> 炽橙 CZY3D 几何内核

这是一个开源的 **内核算法库**，使用 **C++** 实现，旨在为开发者提供高效的算法实现，解决常见的计算和优化问题。该库包含了多种核心算法，具体包括：

**CAD（计算机辅助设计）**：曲线与曲面平滑、拓扑合并与分割等算法，用于提升几何建模的精度和效率。

**CAE（计算机辅助工程）**：网格自适应细分、形状优化等算法，用于在工程分析中生成高质量的网格并进行结构优化。

**CAM（计算机辅助制造）**：加工路径识别、测地线计算等算法，广泛应用于刀具路径计算和3D建模与分析。

### 克隆本项目

```bash
git clone https://github.com/CZY3DKernel/CZY3DKernel.git
cd CZY3DKernel
```
### 目录说明
- `COMMON/`
公共模块目录（通用类型、宏、基础工具、公共数据结构等）。

- `DATA/`
示例/回归测试数据目录，包含典型 CAD 文件（STEP/IGS/stp），用于验证算法输出稳定性与回归对比。

- `GEOM/`
扩展的几何算法模块（可以被`CAM`或其它模块复用）。

- `UTILS/`
工具算法模块（提供好用的开源算法，如KDTree, 用于空间索引/近邻查询/加速检索等）。

- `PIC/`
文档展示图片目录（README/说明文档中引用的效果图、对比图、案例截图）。

- `CAD/`
提供 CAD 几何、拓扑处理相关的核心算法模块，详见CAD内部的ReadMe示例。

- `CAE/`
包含与 CAE 剖分、优化相关的核心算法模块，详见CAE内部的ReadMe示例。

- `CAM/`
包含与 CAM 加工路径相关的核心算法模块，详见CAM内部的ReadMe示例。

## 贡献

欢迎开源社区的贡献！如果你发现任何问题或有新的功能建议，可以通过提交 Issues 或 Pull Requests 参与进来。

### 贡献步骤

1. Fork 本仓库
2. 克隆到本地：`git clone https://github.com/CZY3DKernel/CZY3DKernel.git`
3. 创建一个新的分支：`git checkout -b feature/your-feature-name`
4. 提交你的改动：`git commit -am 'Add new feature or fix bug'`
5. Push 到你的分支：`git push origin feature/your-feature-name`
6. 创建 Pull Request，描述你的改动

## License

 GNU Lesser General Public License version 2.1



