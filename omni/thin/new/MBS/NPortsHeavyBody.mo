within MBS;

model NPortsHeavyBody
  extends RigidBody;

  parameter SI.Acceleration[3] Gravity;
  parameter Integer NPorts = 1;
  WrenchPort[NPorts] InPorts;
equation
  F = m*Gravity + sum(InPorts[i].F for i in 1:NPorts);
  M = sum(InPorts[i].M + cross(InPorts[i].P - r, InPorts[i].F) for i in 1:NPorts);

end NPortsHeavyBody;
