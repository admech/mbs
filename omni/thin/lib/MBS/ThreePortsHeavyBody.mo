within MBS;

model ThreePortsHeavyBody
  extends RigidBody;

  parameter SI.Acceleration[3] Gravity;

  WrenchPort InPort;
  WrenchPort InPort1;
  WrenchPort InPort2;

equation

  F = m * Gravity + InPort.F + InPort1.F + InPort2.F;

  M = InPort.M  + cross(InPort.P  - r, InPort.F )
    + InPort1.M + cross(InPort1.P - r, InPort1.F)
    + InPort2.M + cross(InPort2.P - r, InPort2.F)
    ;

  connect(InPort, InPort);

end ThreePortsHeavyBody;
