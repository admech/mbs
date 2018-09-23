within MBS;

partial model Constraint
  parameter Integer ConstraintNo=1;
  replaceable KinematicPort InPortA annotation (extent=[-40,-100; -20,-80],
      Placement(transformation(extent={{-60,-100},{-20,-60}})));
  replaceable WrenchPort OutPortA annotation (extent=[-40,80; -20,100],
      Placement(transformation(extent={{-60,60},{-20,100}})));
  replaceable KinematicPort InPortB annotation (extent=[20,-100; 40,-80],
      Placement(transformation(extent={{20,-100},{60,-60}})));
  replaceable WrenchPort OutPortB annotation (extent=[20,80; 40,100],
      Placement(transformation(extent={{20,60},{60,100}})));
  annotation (Icon, Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
            -100},{100,100}}),       graphics));
equation
  OutPortA.F + OutPortB.F = zeros(3);
  OutPortA.M + OutPortB.M = zeros(3);
end Constraint;
