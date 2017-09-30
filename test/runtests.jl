using Gurobi, Base.Test, MathOptInterface, MathOptInterfaceGurobi
include(joinpath(Pkg.dir("MathOptInterface"), "test", "contlinear.jl"))
include(joinpath(Pkg.dir("MathOptInterface"), "test", "intlinear.jl"))
include(joinpath(Pkg.dir("MathOptInterface"), "test", "contconic.jl"))
include(joinpath(Pkg.dir("MathOptInterface"), "test", "contquadratic.jl"))


# contlinear
linear1test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0))
linear2test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0))
linear3test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0))
linear4test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0))
linear5test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0))
linear6test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0))
linear7test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0))
linear8test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0)) # infeasible/unbounded
linear9test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0))
# linear10test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0)) # ranged
linear11test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0))

# # intlinear
knapsacktest(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0))
int1test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0))
int2test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0)) # SOS
# int3test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0)) # ranged

# # contconic
lin1tests(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0))
lin2tests(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0))
lin3test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0,InfUnbdInfo = 1)) # infeasible
lin4test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0,InfUnbdInfo = 1)) # infeasible

# # contquadratic
qp1test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0), atol = 1e-5)
qp2test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0), atol = 1e-5)
qp3test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0), atol = 1e-5)
qcp1test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0), atol = 1e-3)
# qcp2test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0), atol = 1e-5) # duals
# qcp3test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0), atol = 1e-5) # duals
socp1test(MathOptInterfaceGurobi.MOIGurobiSolver(OutputFlag=0), atol = 1e-5)
