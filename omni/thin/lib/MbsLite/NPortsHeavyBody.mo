within MbsLite;

model NPortsHeavyBody
  extends RigidBody;

  parameter Integer N = -Integer_inf;
  parameter SI.Acceleration[3] Gravity = fill(inf, 3);

  WrenchPort[N] InPorts;

  Real[N, 3] crosses (each stateSelect = StateSelect.never);

initial algorithm
  AssertInitializedI(name, N,       "N");
  AssertInitialized (name, Gravity, "Gravity");

equation

  crosses = { cross(InPorts[i].P - r, InPorts[i].F) for i in 1 : N };

  F = m * Gravity + { sum(InPorts.F[i]) for i in 1 : 3 };

  M = { sum(InPorts.M[i] + crosses[:,i]) for i in 1 : 3 };

  // FIXME: WHY WAS THIS NEEDED ? THE COMPILER IGNORES IT ANYWAY
  // connect(InPorts[1], InPorts[1]);

end NPortsHeavyBody;
