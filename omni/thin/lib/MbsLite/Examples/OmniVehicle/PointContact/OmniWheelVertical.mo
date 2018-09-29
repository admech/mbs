within MbsLite.Examples.OmniVehicle.PointContact;

model OmniWheelVertical

  parameter Real R = 1;
  parameter Integer n = 5;
  
  Base base;

  OmniWheelGeneral wheel
    ( name    = "wheel"
    , n       = n
    , rollerMass          = 1
    , rollerAxialMoi      = 1
    , rollerOrthogonalMoi = 1
    , R       = R
    , r0      = { 0, R, 0 }
    , q0      = QRot(0, { 0, 0, 0 })
    , v0      = { 0, 0, 0 }
    , omega0  = { 0, 0, 0 }
    );

equation

/*
  for i in 1 : n loop
    connect(base.OutPort, wheel.Contacts[i].InPortA);
    connect(body.OutPort, wheel.Contacts[i].InPortB);
    connect(body.InPorts[1], joint.OutPortB);
  end for;
*/

end OmniWheelVertical;
