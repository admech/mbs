within MbsLite.Examples.OmniVehicle.PointContact;

model OmniWheelOnPlane
  extends OmniWheel;

  RollerPointContactForcesGeneral[nActual] Contacts
      ( name       = { name + ".Contacts[" + String(i) + "]" for i in 1 : nActual }
      , each n     = params.nRollers
      , each R     = params.wheelRadius
      , each psi   = params.mecanumAngle
      , each alpha = params.rollerHalfAngle
      , each R1    = params.wheelHubRadius
      , each L1    = params.rollerLength
      , each nA    = vertical
      );

  KinematicPort  InPortK   "takes kinematics from base (which has only an OutPort and which needn't import any forces as it has no dynamics)";

equation

  for i in 1 : nActual loop
    Contacts[i].n1k = Wheel.T * { 0, 0, 1 };
    Contacts[i].rho = (Wheel.r - Rollers[i].r) / sqrt((Wheel.r - Rollers[i].r) * (Wheel.r - Rollers[i].r));

    // interactions between Rollers and base (the floor) -- via Contacts
    connect(InPortK,                 Contacts[i].InPortA);  // kinematics from base (floor) goes in via InPortK and is being passed into Contacts via their InPortA
    connect(Rollers[i].OutPort,      Contacts[i].InPortB);  // similarly, kinematics from Rollers is fed in into Contacts via InPortB
    connect(Rollers[i].InPorts[1],   Contacts[i].OutPortB); // then what forces are calculated within Contacts, are being exported to Rollers from the Contacts' OutPortB and via the Rollers' first InPorts
  end for;

end OmniWheelOnPlane;

