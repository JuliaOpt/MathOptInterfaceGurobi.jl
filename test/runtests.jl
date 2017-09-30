using Gurobi, Base.Test, MathOptInterface, GurobiMathOptInterface
include(joinpath(Pkg.dir("MathOptInterface"), "test", "contlinear.jl"))
include(joinpath(Pkg.dir("MathOptInterface"), "test", "intlinear.jl"))
include(joinpath(Pkg.dir("MathOptInterface"), "test", "contconic.jl"))
include(joinpath(Pkg.dir("MathOptInterface"), "test", "contquadratic.jl"))


# contlinear
linear1test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0))
linear2test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0))
linear3test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0))
linear4test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0))
linear5test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0))
linear6test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0))
linear7test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0))
linear8test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0)) # infeasible/unbounded
linear9test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0))
# linear10test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0)) # ranged
linear11test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0))

# # intlinear
knapsacktest(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0))
int1test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0))
int2test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0)) # SOS
# int3test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0)) # ranged

# # contconic
lin1tests(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0))
lin2tests(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0))
# lin3test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0,InfUnbdInfo = 1)) # infeasible
# lin4test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0,InfUnbdInfo = 1)) # infeasible

# # contquadratic
qp1test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0), atol = 1e-5)
qp2test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0), atol = 1e-5)
qp3test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0), atol = 1e-5)
qcp1test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0), atol = 1e-3)
# qcp2test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0), atol = 1e-5) # duals
# qcp3test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0), atol = 1e-5) # duals
socp1test(GurobiMathOptInterface.MOIGurobiSolver(OutputFlag=0), atol = 1e-5)
