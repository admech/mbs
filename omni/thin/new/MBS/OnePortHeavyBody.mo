within MBS;

model OnePortHeavyBody
  extends RigidBody;
  parameter SI.Acceleration[3] Gravity;
  WrenchPort InPort 
    annotation (Placement(transformation(extent={{-20,60},{20,100}})));
equation
  F = m*Gravity + InPort.F;
  M = InPort.M + cross(InPort.P - r, InPort.F);
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}),      graphics));
end OnePortHeavyBody;
