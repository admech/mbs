within MbsLite;

model NPortsHeavyBody
  extends RigidBody;

  parameter Integer N;
  parameter SI.Acceleration[3] Gravity;

  WrenchPort[N] InPorts;

protected
  Real[N, 3] crosses;

equation

  crosses = { cross(InPorts[i].P - r, InPorts[i].F) for i in 1 : N };

  F = m * Gravity + { sum(InPorts.F[i]) for i in 1 : 3 };

  M = { sum(InPorts.M[i] + crosses[:,i]) for i in 1 : 3 };

  connect(InPorts[1], InPorts[1]);

end NPortsHeavyBody;
