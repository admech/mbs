within MBS.Examples.OmniVehicle;

model ThreeWheelsVehicle
  OmniWheel Wheel1 
    annotation (Placement(transformation(extent={{-46,40},{-6,80}})));
  OmniWheel Wheel2 
    annotation (Placement(transformation(extent={{-46,-20},{-6,20}})));
  OmniWheel Wheel3 
    annotation (Placement(transformation(extent={{-46,-80},{-6,-40}})));
  Base Floor 
            annotation (Placement(transformation(extent={{-96,-20},{-56,
            20}})));
  FixedJoint Joint1(
    nA = {1, 0, 0},
    nB = {0, 1, 0},
    rA = {0, 0, 0},
    rB = {R1, 0, 0}) 
    annotation (Placement(transformation(extent={{4,40},{44,80}})));
  FixedJoint Joint2(
    nA = {1, 0, 0},
    nB = {0, 1, 0},
    rA = {0, 0, 0},
    rB = {R1, 0, 0}) 
    annotation (Placement(transformation(extent={{4,-20},{44,20}})));
  FixedJoint Joint3(
    nA = {1, 0, 0},
    nB = {0, 1, 0},
    rA = {0, 0, 0},
    rB = {R1, 0, 0}) 
    annotation (Placement(transformation(extent={{4,-80},{44,-40}})));
  ThreePortsHeavyBody Body 
    annotation (Placement(transformation(extent={{54,-20},{94,20}})));
  annotation (Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
            -100},{100,100}}),       graphics));
equation
  connect(Wheel1.InPortK, Floor.OutPort) annotation (Line(
      points={{-34,42},{-34,34},{-52,34},{-52,-26},{-76,-26},{-76,-16}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel2.InPortK, Floor.OutPort) annotation (Line(
      points={{-34,-18},{-34,-26},{-76,-26},{-76,-16}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel3.InPortK, Floor.OutPort) annotation (Line(
      points={{-34,-78},{-34,-86},{-52,-86},{-52,-26},{-76,-26},{-76,-16}},
      color={0,0,255},
      smooth=Smooth.None));

  connect(Wheel1.InPortF, Joint1.OutPortA) annotation (Line(
      points={{-18,78},{-18,86},{16,86},{16,76}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel2.InPortF, Joint2.OutPortA) annotation (Line(
      points={{-18,18},{-18,26},{16,26},{16,16}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel3.InPortF, Joint3.OutPortA) annotation (Line(
      points={{-18,-42},{-18,-34},{16,-34},{16,-44}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel3.OutPortK, Joint3.InPortA) annotation (Line(
      points={{-18,-78},{-18,-86},{16,-86},{16,-76}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel2.OutPortK, Joint2.InPortA) annotation (Line(
      points={{-18,-18},{-18,-26},{16,-26},{16,-16}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Wheel1.OutPortK, Joint1.InPortA) annotation (Line(
      points={{-18,42},{-18,34},{16,34},{16,44}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Joint2.OutPortB, Body.InPort) annotation (Line(
      points={{32,16},{32,26},{62,26},{62,16}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Joint1.OutPortB, Body.InPort1) annotation (Line(
      points={{32,76},{32,86},{74,86},{74,16}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Joint3.OutPortB, Body.InPort2) annotation (Line(
      points={{32,-44},{32,-34},{96,-34},{96,26},{86,26},{86,16}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Body.OutPort, Joint2.InPortB) annotation (Line(
      points={{74,-16},{74,-26},{32,-26},{32,-16}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Body.OutPort, Joint1.InPortB) annotation (Line(
      points={{74,-16},{74,-26},{48,-26},{48,34},{32,34},{32,44}},
      color={0,0,255},
      smooth=Smooth.None));
  connect(Body.OutPort, Joint3.InPortB) annotation (Line(
      points={{74,-16},{74,-26},{48,-26},{48,-86},{32,-86},{32,-76}},
      color={0,0,255},
      smooth=Smooth.None));
end ThreeWheelsVehicle;
