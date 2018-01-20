module MathOptInterfaceGurobi
# todo
# single get/set
# fix chg coeff
# inplace getters
# updates!

export GurobiSolverInstance

const GRBMOI = MathOptInterfaceGurobi

import Base.show, Base.copy

using Gurobi
const GRB = Gurobi
using MathOptInterface
const MOI = MathOptInterface
using LinQuadOptInterface
const LQOI = LinQuadOptInterface

const SUPPORTED_OBJECTIVES = [
    LQOI.Linear,
    LQOI.Quad
]
const SUPPORTED_CONSTRAINTS = [
    (LQOI.Linear, LQOI.EQ),
    (LQOI.Linear, LQOI.LE),
    (LQOI.Linear, LQOI.GE),
    # (Linear, IV),
    (LQOI.Quad, LQOI.EQ),
    (LQOI.Quad, LQOI.LE),
    (LQOI.Quad, LQOI.GE),
    (LQOI.SinVar, LQOI.EQ),
    (LQOI.SinVar, LQOI.LE),
    (LQOI.SinVar, LQOI.GE),
    (LQOI.SinVar, LQOI.IV),
    (LQOI.SinVar, MOI.ZeroOne),
    (LQOI.SinVar, MOI.Integer),
    (LQOI.VecVar, MOI.SOS1),
    (LQOI.VecVar, MOI.SOS2),
    (LQOI.VecVar, MOI.Nonnegatives),
    (LQOI.VecVar, MOI.Nonpositives),
    (LQOI.VecVar, MOI.Zeros),
    (LQOI.VecLin, MOI.Nonnegatives),
    (LQOI.VecLin, MOI.Nonpositives),
    (LQOI.VecLin, MOI.Zeros)
]

import Gurobi.Model

function GRB.Model(env::GRB.Env)
    return GRB.Model(env,"defaultname")
end

mutable struct GurobiSolverInstance <: LQOI.LinQuadSolverInstance
    LQOI.@LinQuadSolverInstanceBase
    env
end
function GurobiSolverInstance(;kwargs...)

    env = GRB.Env()
    instance = GurobiSolverInstance(
        (LQOI.@LinQuadSolverInstanceBaseInit)...,
        env
    )
    for (name,value) in kwargs
        GRB.setparam!(instance.inner, string(name), value)
    end
    # csi.inner.mipstart_effort = s.mipstart_effortlevel
    # if s.logfile != ""
    #     LQOI.lqs_setlogfile!(env, s.logfile)
    # end
    return instance
end

LQOI.lqs_supported_constraints(s::GurobiSolverInstance) = SUPPORTED_CONSTRAINTS
LQOI.lqs_supported_objectives(s::GurobiSolverInstance) = SUPPORTED_OBJECTIVES
#=
    inner wrapper
=#

#=
    Main
=#

# LinQuadSolver # Abstract type
# done above

# LQOI.lqs_setparam!(env, name, val)
# TODO fix this one
LQOI.lqs_setparam!(m::GurobiSolverInstance, name, val) = GRB.setparam!(m.inner, string(name), val)

# LQOI.lqs_setlogfile!(env, path)
# TODO fix this one
LQOI.lqs_setlogfile!(m::GurobiSolverInstance, path) = GRB.setlogfile(m.env, path::String)

# LQOI.lqs_getprobtype(m)
# TODO - consider removing, apparently useless

#=
    Constraints
=#

cintvec(v::Vector) = convert(Vector{Int32}, v)
cdoublevec(v::Vector) = convert(Vector{Float64}, v)

_getsense(m::GurobiSolverInstance, ::MOI.EqualTo{Float64}) = Cchar('=')
_getsense(m::GurobiSolverInstance, ::MOI.LessThan{Float64}) = Cchar('<')
_getsense(m::GurobiSolverInstance, ::MOI.GreaterThan{Float64}) = Cchar('>')
_getsense(m::GurobiSolverInstance, ::MOI.Zeros)        = Cchar('=')
_getsense(m::GurobiSolverInstance, ::MOI.Nonpositives) = Cchar('<')
_getsense(m::GurobiSolverInstance, ::MOI.Nonnegatives) = Cchar('>')
_getboundsense(m::GurobiSolverInstance, ::MOI.Nonpositives) = Cchar('>')
_getboundsense(m::GurobiSolverInstance, ::MOI.Nonnegatives) = Cchar('<')

# LQOI.lqs_chgbds!(m, colvec, valvec, sensevec)
# TODO - improve single type
function LQOI.lqs_chgbds!(instance::GurobiSolverInstance, colvec, valvec, sensevec)
    lb_len = count(x->x==Cchar('L'), sensevec)
    LB_val = Array{Float64}(0)
    sizehint!(LB_val, lb_len)
    LB_col = Array{Cint}(0)
    sizehint!(LB_col, lb_len)
    
    ub_len = count(x->x==Cchar('U'), sensevec)
    UB_val = Array{Float64}(0)
    sizehint!(UB_val, ub_len)
    UB_col = Array{Cint}(0)
    sizehint!(UB_col, ub_len)

    for i in eachindex(valvec)
        if sensevec[i] == Cchar('L')
            push!(LB_col, colvec[i])
            push!(LB_val, valvec[i])
        elseif sensevec[i] == Cchar('U')
            push!(UB_col, colvec[i])
            push!(UB_val, valvec[i])
        end
    end

    if lb_len > 0
        GRB.set_dblattrlist!(instance.inner, "LB", LB_col, LB_val)
    end
    
    if ub_len > 0
        GRB.set_dblattrlist!(instance.inner, "UB", UB_col, UB_val)
    end

    GRB.update_model!(instance.inner)
    nothing
end


# LQOI.lqs_getlb(m, col)
LQOI.lqs_getlb(instance::GurobiSolverInstance, col) = (GRB.update_model!(instance.inner);GRB.get_dblattrlist( instance.inner, "LB", GRB.ivec(col))[1])
# LQOI.lqs_getub(m, col)
LQOI.lqs_getub(instance::GurobiSolverInstance, col) = GRB.get_dblattrlist( instance.inner, "UB", GRB.ivec(col))[1]

# LQOI.lqs_getnumrows(m)
LQOI.lqs_getnumrows(instance::GurobiSolverInstance) = GRB.num_constrs(instance.inner)

# LQOI.lqs_addrows!(m, rowvec, colvec, coefvec, sensevec, rhsvec)
LQOI.lqs_addrows!(instance::GurobiSolverInstance, rowvec, colvec, coefvec, sensevec, rhsvec) = (GRB.add_constrs!(instance.inner, rowvec, colvec, coefvec, sensevec, rhsvec);GRB.update_model!(instance.inner))

# LQOI.lqs_getrhs(m, rowvec)
LQOI.lqs_getrhs(instance::GurobiSolverInstance, row) = GRB.get_dblattrlist( instance.inner, "RHS", GRB.ivec(row))[1]

# colvec, coef = LQOI.lqs_getrows(m, rowvec)
# TODO improve
function LQOI.lqs_getrows(instance::GurobiSolverInstance, idx)
    A = GRB.get_constrs(instance.inner, idx, 1)'
    return A.rowval-1, A.nzval
end

# LQOI.lqs_getcoef(m, row, col) #??
# TODO improve
function LQOI.lqs_getcoef(instance::GurobiSolverInstance, row, col) #??
    return getcoeff(model::Model, row::Integer, col::Integer)
    # A = GRB.get_rows(m, row, row)'
    # cols = A.rowval
    # vals = A.nzval

    # pos = findfirst(cols, col)
    # if pos > 0
    #     return vals[pos]
    # else
    #     return 0.0
    # end
end

# LQOI.lqs_chgcoef!(m, row, col, coef)
# TODO SPLIT THIS ONE
function LQOI.lqs_chgcoef!(instance::GurobiSolverInstance, row, col, coef) 
    if row == 0
        GRB.set_dblattrlist!(instance.inner, "Obj", Cint[col], Float64[coef])
    elseif col == 0
        GRB.set_dblattrlist!(instance.inner, "RHS", Cint[row], Float64[coef])
    else
        GRB.chg_coeffs!(instance.inner, row, col, coef)
        #TODO fix this function in gurobi
    end
end

# LQOI.lqs_delrows!(m, row, row)
LQOI.lqs_delrows!(instance::GurobiSolverInstance, rowbeg, rowend) = GRB.del_constrs!(instance.inner, cintvec(collect(rowbeg:rowend))) 

# LQOI.lqs_chgctype!(m, colvec, typevec)
# TODO fix types
LQOI.lqs_chgctype!(instance::GurobiSolverInstance, colvec, typevec) = GRB.set_charattrlist!(instance.inner, "VType", GRB.ivec(colvec), GRB.cvec(typevec))

# LQOI.lqs_chgsense!(m, rowvec, sensevec)
# TODO fix types
LQOI.lqs_chgsense!(instance::GurobiSolverInstance, rowvec, sensevec) = GRB.set_charattrlist!(instance.inner, "Sense", GRB.ivec(rowvec), GRB.cvec(sensevec))

const VAR_TYPE_MAP = Dict{Symbol,Cchar}(
    :CONTINUOUS => Cchar('C'),
    :INTEGER => Cchar('I'),
    :BINARY => Cchar('B')
)
LQOI.lqs_vartype_map(m::GurobiSolverInstance) = VAR_TYPE_MAP

# LQOI.lqs_addsos(m, colvec, valvec, typ)
LQOI.lqs_addsos!(instance::GurobiSolverInstance, colvec, valvec, typ) = (GRB.add_sos!(instance.inner, typ, colvec, valvec);GRB.update_model!(instance.inner))
# LQOI.lqs_delsos(m, idx, idx)
LQOI.lqs_delsos!(instance::GurobiSolverInstance, idx1, idx2) = (GRB.del_sos!(instance.inner, cintvec(collect(idx1:idx2)));GRB.update_model!(instance.inner))

const SOS_TYPE_MAP = Dict{Symbol,Symbol}(
    :SOS1 => :SOS1,#Cchar('1'),
    :SOS2 => :SOS2#Cchar('2')
)
LQOI.lqs_sertype_map(m::GurobiSolverInstance) = SOS_TYPE_MAP

# LQOI.lqs_getsos(m, idx)
# TODO improve getting processes
function LQOI.lqs_getsos(instance::GurobiSolverInstance, idx)
    A, types = GRB.get_sos_matrix(instance.inner)
    line = A[idx,:] #sparse vec
    cols = line.nzind
    vals = line.nzval
    typ = types[idx] == Cint(1) ? :SOS1 : :SOS2
    return cols, vals, typ
end

# LQOI.lqs_getnumqconstrs(m)
LQOI.lqs_getnumqconstrs(instance::GurobiSolverInstance) = GRB.num_qconstrs(instance.inner)

# LQOI.lqs_addqconstr(m, cols,coefs,rhs,sense, I,J,V)
LQOI.lqs_addqconstr!(instance::GurobiSolverInstance, cols,coefs,rhs,sense, I,J,V) = GRB.add_qconstr!(instance.inner, cols, coefs, I, J, V, sense, rhs)

# LQOI.lqs_chgrngval
LQOI.lqs_chgrngval!(instance::GurobiSolverInstance, rows, vals) = GRB.chg_rhsrange!(instance.inner, cintvec(rows), -vals)

const CTR_TYPE_MAP = Dict{Symbol,Cchar}(
    :RANGE => Cchar('R'),
    :LOWER => Cchar('L'),
    :UPPER => Cchar('U'),
    :EQUALITY => Cchar('E')
)
LQOI.lqs_ctrtype_map(m::GurobiSolverInstance) = CTR_TYPE_MAP

#=
    Objective
=#

# LQOI.lqs_copyquad(m, intvec,intvec, floatvec) #?
function LQOI.lqs_copyquad!(instance::GurobiSolverInstance, I, J, V)
    GRB.delq!(instance.inner)
    for i in eachindex(V)
        if I[i] == J[i]
            V[i] /= 2
        end
    end
    GRB.add_qpterms!(instance.inner, I, J, V)
    return nothing
end

# LQOI.lqs_chgobj(m, colvec,coefvec)
function LQOI.lqs_chgobj!(instance::GurobiSolverInstance, colvec, coefvec) 
    nvars = GRB.num_vars(instance.inner)
    obj = zeros(Float64, nvars)

    for i in eachindex(colvec)
        obj[colvec[i]] = coefvec[i]
    end

    GRB.set_dblattrarray!(instance.inner, "Obj", 1, num_vars(instance.inner), obj)
    GRB.update_model!(instance.inner)
    nothing
end

# LQOI.lqs_chgobjsen(m, symbol)
# TODO improve min max names
function LQOI.lqs_chgobjsen!(instance::GurobiSolverInstance, symbol)
    if symbol in [:minimize, :Min]
        GRB.set_sense!(instance.inner, :minimize)
    else
        GRB.set_sense!(instance.inner, :maximize)
    end
    GRB.update_model!(instance.inner)
end
    

# LQOI.lqs_getobj(instance.inner)
LQOI.lqs_getobj(instance::GurobiSolverInstance) = GRB.get_dblattrarray( instance.inner, "Obj", 1, num_vars(instance.inner)   )

# lqs_getobjsen(m)
function LQOI.lqs_getobjsen(instance::GurobiSolverInstance)
    s = GRB.model_sense(instance.inner)
    if s in [:maximize, :Max]
        return MOI.MaxSense
    else
        return MOI.MinSense
    end
end

#=
    Variables
=#

# LQOI.lqs_getnumcols(m)
LQOI.lqs_getnumcols(instance::GurobiSolverInstance) = (GRB.update_model!(instance.inner); GRB.num_vars(instance.inner))

# LQOI.lqs_newcols!(m, int)
LQOI.lqs_newcols!(instance::GurobiSolverInstance, int) = (GRB.add_cvars!(instance.inner, zeros(int));GRB.update_model!(instance.inner))

# LQOI.lqs_delcols!(m, col, col)
LQOI.lqs_delcols!(instance::GurobiSolverInstance, col, col2) = (GRB.del_vars!(instance.inner, col);GRB.update_model!(instance.inner))

# LQOI.lqs_addmipstarts(m, colvec, valvec)
function LQOI.lqs_addmipstarts!(instance::GurobiSolverInstance, colvec, valvec) 
    x = zeros(GRB.num_vars(instance.inner))
    for i in eachindex(colvec)
        x[colvec[i]] = valvec[i]
    end
    GRB.loadbasis(instance.inner, x)
end
#=
    Solve
=#

# LQOI.lqs_mipopt!(m)
LQOI.lqs_mipopt!(instance::GurobiSolverInstance) = LQOI.lqs_lpopt!(instance)

# LQOI.lqs_qpopt!(m)
LQOI.lqs_qpopt!(instance::GurobiSolverInstance) = LQOI.lqs_lpopt!(instance)

# LQOI.lqs_lpopt!(m)
LQOI.lqs_lpopt!(instance::GurobiSolverInstance) = (GRB.update_model!(instance.inner);GRB.optimize(instance.inner))

# LQOI.lqs_terminationstatus(m)
function LQOI.lqs_terminationstatus(instance::GurobiSolverInstance)
    
    stat = get_status(instance.inner)

    if stat == :loaded
        return MOI.OtherError
    elseif stat == :optimal
        return MOI.Success
    elseif stat == :infeasible
        if hasdualray(instance)
            return MOI.Success
        else
            return MOI.InfeasibleNoResult
        end
    elseif stat == :inf_or_unbd
        return MOI.InfeasibleOrUnbounded
    elseif stat == :unbounded
        if hasprimalray(instance)
            return MOI.Success
        else
            return MOI.UnboundedNoResult
        end
    elseif stat == :cutoff
        return MOI.ObjectiveLimit
    elseif stat == :iteration_limit
        return MOI.IterationLimit
    elseif stat == :node_limit
        return MOI.NodeLimit
    elseif stat == :time_limit
        return MOI.TimeLimit
    elseif stat == :solution_limit
        return MOI.SolutionLimit
    elseif stat == :interrupted
        return MOI.Interrupted
    elseif stat == :numeric
        return MOI.NumericalError
    elseif stat == :suboptimal
        return MOI.OtherLimit
    elseif stat == :inprogress
        return MOI.OtherError
    elseif stat == :user_obj_limit
        return MOI.ObjectiveLimit
    end
    return MOI.OtherError
end


function LQOI.lqs_primalstatus(instance::GurobiSolverInstance)

    stat = get_status(instance.inner)

    if stat == :optimal
        return MOI.FeasiblePoint
    elseif stat == :solution_limit
        return MOI.FeasiblePoint
    elseif stat in [:inf_or_unbd, :unbounded] && hasprimalray(instance)
        return MOI.InfeasibilityCertificate
    elseif stat == :suboptimal
        return MOI.FeasiblePoint
    else 
        return MOI.UnknownResultStatus
    end
end
function LQOI.lqs_dualstatus(instance::GurobiSolverInstance)
    stat = get_status(instance.inner)
    
    if GRB.is_mip(instance.inner) || GRB.is_qcp(instance.inner)
        return MOI.UnknownResultStatus
    else
        if stat == :optimal
            return MOI.FeasiblePoint
        elseif stat == :solution_limit
            return MOI.FeasiblePoint
        elseif stat in [:inf_or_unbd, :infeasible] && hasdualray(instance)
            return MOI.InfeasibilityCertificate
        elseif stat == :suboptimal
            return MOI.FeasiblePoint
        else 
            return MOI.UnknownResultStatus
        end  
    end
end


# LQOI.lqs_getx!(m, place)
LQOI.lqs_getx!(instance::GurobiSolverInstance, place) = GRB.get_dblattrarray!(place, instance.inner, "X", 1)

# LQOI.lqs_getax!(m, place)
function LQOI.lqs_getax!(instance::GurobiSolverInstance, place)
    GRB.get_dblattrarray!(place, instance.inner, "Slack", 1)
    rhs = GRB.get_dblattrarray(instance.inner, "RHS", 1, num_constrs(instance.inner))
    for i in eachindex(place)
        place[i] = -place[i]+rhs[i]
    end
    nothing
end
# LQOI.lqs_getdj!(m, place)
LQOI.lqs_getdj!(instance::GurobiSolverInstance, place) = GRB.get_dblattrarray!(place, instance.inner, "RC", 1)

# LQOI.lqs_getpi!(m, place)
LQOI.lqs_getpi!(instance::GurobiSolverInstance, place) = GRB.get_dblattrarray!(place, instance.inner, "Pi", 1)

# LQOI.lqs_getobjval(m)
LQOI.lqs_getobjval(instance::GurobiSolverInstance) = GRB.get_objval(instance.inner)

# LQOI.lqs_getbestobjval(m)
LQOI.lqs_getbestobjval(instance::GurobiSolverInstance) = GRB.get_objval(instance.inner)

# LQOI.lqs_getmiprelgap(m)
function LQOI.lqs_getmiprelgap(instance::GurobiSolverInstance)
    L = GRB.get_objval(instance.inner)
    U = GRB.get_objbound(instance.inner)
    return abs(U-L)/U
end

# LQOI.lqs_getitcnt(m)
LQOI.lqs_getitcnt(instance::GurobiSolverInstance)  = GRB.get_iter_count(instance.inner)

# LQOI.lqs_getbaritcnt(m)
LQOI.lqs_getbaritcnt(instance::GurobiSolverInstance) = GRB.get_barrier_iter_count(instance.inner)

# LQOI.lqs_getnodecnt(m)
LQOI.lqs_getnodecnt(instance::GurobiSolverInstance) = GRB.get_node_count(instance.inner)

# LQOI.lqs_dualfarkas(m, place)
LQOI.lqs_dualfarkas!(instance::GurobiSolverInstance, place) = GRB.get_dblattrarray!(place, instance.inner, "FarkasDual", 1)

function hasdualray(instance::GurobiSolverInstance)
    try
        GRB.get_dblattrarray(instance.inner, "FarkasDual", 1, GRB.num_constrs(instance.inner))
        return true
    catch
        return false
    end
end

# LQOI.lqs_getray(m, place)
LQOI.lqs_getray!(instance::GurobiSolverInstance, place) = GRB.get_dblattrarray!(place, instance.inner, "UnbdRay", 1)

function hasprimalray(instance::GurobiSolverInstance)
    try
        GRB.get_dblattrarray(instance.inner, "UnbdRay", 1, GRB.num_vars(instance.inner))
        return true
    catch
        return false
    end
end

MOI.free!(m::GurobiSolverInstance) = GRB.free_model(m.inner)

"""
    writeproblem(m::AbstractSolverInstance, filename::String)
Writes the current problem data to the given file.
Supported file types are solver-dependent.
"""
writeproblem(m::GurobiSolverInstance, filename::String, flags::String="") = GRB.write_model(m.inner, filename)


# blocked
MOI.addconstraint!(m::GurobiSolverInstance, func::LQOI.Linear, set::LQOI.IV) = error("not supported")
MOI.addconstraints!(m::GurobiSolverInstance, func::Vector{LQOI.Linear}, set::Vector{LQOI.IV}) = error("not supported")

MOI.canget(m::GurobiSolverInstance, any, c::LQOI.LCI{LQOI.IV}) = false
MOI.canmodifyconstraint(m::GurobiSolverInstance, c::LQOI.LCI{LQOI.IV}, chg) = false
MOI.candelete(m::GurobiSolverInstance, c::LQOI.LCI{LQOI.IV}) = false

end # module