using Gurobi, Base.Test, MathOptInterface, GurobiMathOptInterface
include(joinpath(Pkg.dir("MathOptInterface"), "test", "contlinear.jl"))
include(joinpath(Pkg.dir("MathOptInterface"), "test", "intlinear.jl"))
include(joinpath(Pkg.dir("MathOptInterface"), "test", "contconic.jl"))
include(joinpath(Pkg.dir("MathOptInterface"), "test", "contquadratic.jl"))


# contlinear
linear1test(GurobiMathOptInterface.MOIGurobiSolver())
linear2test(GurobiMathOptInterface.MOIGurobiSolver())
linear3test(GurobiMathOptInterface.MOIGurobiSolver())
linear4test(GurobiMathOptInterface.MOIGurobiSolver())
linear5test(GurobiMathOptInterface.MOIGurobiSolver())
linear6test(GurobiMathOptInterface.MOIGurobiSolver())
linear7test(GurobiMathOptInterface.MOIGurobiSolver())
# linear8test(GurobiMathOptInterface.MOIGurobiSolver()) # infeasible/unbounded
linear9test(GurobiMathOptInterface.MOIGurobiSolver())
# linear10test(GurobiMathOptInterface.MOIGurobiSolver()) # ranged
linear11test(GurobiMathOptInterface.MOIGurobiSolver())

# # intlinear
knapsacktest(GurobiMathOptInterface.MOIGurobiSolver())
int1test(GurobiMathOptInterface.MOIGurobiSolver())
int2test(GurobiMathOptInterface.MOIGurobiSolver()) # SOS
# int3test(GurobiMathOptInterface.MOIGurobiSolver()) # ranged

# # contconic
lin1tests(GurobiMathOptInterface.MOIGurobiSolver())
lin2tests(GurobiMathOptInterface.MOIGurobiSolver())
# lin3test(GurobiMathOptInterface.MOIGurobiSolver() # infeasible
# lin4test(GurobiMathOptInterface.MOIGurobiSolver()) # infeasible

# # contquadratic
# qp1test(GurobiMathOptInterface.MOIGurobiSolver(), atol = 1e-5)
# qp2test(GurobiMathOptInterface.MOIGurobiSolver(), atol = 1e-5)
# qp3test(GurobiMathOptInterface.MOIGurobiSolver(), atol = 1e-5)
# qcp1test(GurobiMathOptInterface.MOIGurobiSolver(), atol = 1e-5)
# qcp2test(GurobiMathOptInterface.MOIGurobiSolver(), atol = 1e-5)
# qcp3test(GurobiMathOptInterface.MOIGurobiSolver(), atol = 1e-5)
# socp1test(GurobiMathOptInterface.MOIGurobiSolver(), atol = 1e-5)
