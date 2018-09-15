within MBS;

model OnePortBody
  extends RigidBody;
  WrenchPort InPort 
    annotation (Placement(transformation(extent={{-20,60},{20,100}})));
equation
  F = InPort.F;
  M = InPort.M + cross(InPort.P - r, InPort.F);
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}}),      graphics));
end OnePortBody;

