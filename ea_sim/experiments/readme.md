# Experiments of Evolution of Tensegrity Soft Modular Robots

In this folder are reported the configuration of each experiment that has been carried out by the original project. In the table below are briefly reported the main characteristics that differentiate each test.

<div class="center">

| **Experiment ID** |  **Algorithm** | **Stiffness** | **White Noisy Level** |
|:-----------------:|:--------------:|:-------------:|:---------------------:|
|         1         |       ViE      |       xS      |          0            |
|         2         |       ViE      |      0.06     |          0            |
|         3         |       ViE      |      0.2      |          0            |
|         4         |       ViE      |      2.0      |          0            |
|         5         |       ViE      |       S       |          0.035        |
|         6         |       ViE      |      0.1      |          0.035        |
|         7         |       ViE      |      0.5      |          0.035        |
|         8         |       ViE      |      1.0      |          0.035        |
|         9         |       μ+λ      |       xS      |          0            |
|         10        |       μ+λ      |      0.06     |          0            |
|         11        |       μ+λ      |      0.2      |          0            |
|         12        |       μ+λ      |      2.0      |          0            |
|         13        |       μ+λ      |       S       |          0.035        |
|         14        |       μ+λ      |      0.1      |          0.035        |
|         15        |       μ+λ      |      0.5      |          0.035        |
|         16        |       μ+λ      |      1.0      |          0.035        |
|         17        |   MAP-Elites   |       xS      |          0            |
|         18        |   MAP-Elites   |       S       |          0.035        |

**Note:** *selected stiffness* means that the stiffness value is randomly chosen by the algorithm, among the available ones, during the evolution process.

</div>

#### Other parameters details
Each simulated robot is composed by a number of modules, each of which is characterized by:
+ actuator signal frequency `[0.25, 0.5]`
+ actuator signal amplitude `[0.6]`
+ actuator signal phase `[0, 1.57, 3.14, 4.71]`
+ faces orientation (rotation) `[0-3]`
+ stiffness value either selected from the standard range S `[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1]` or from the extended one xS `[0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 2, 3, 4]`