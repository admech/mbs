within MbsLite;

model Constraint

  replaceable KinematicPort  InPortA;
  replaceable WrenchPort     OutPortA;
  replaceable KinematicPort  InPortB;
  replaceable WrenchPort     OutPortB;

equation

  OutPortA.F + OutPortB.F = zeros(3);
  OutPortA.M + OutPortB.M = zeros(3);

end Constraint;
