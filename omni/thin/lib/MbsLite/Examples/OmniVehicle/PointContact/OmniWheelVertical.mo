within MbsLite.Examples.OmniVehicle.PointContact;

model OmniWheelVertical

  parameter Real R = 1;
  
  OmniWheelGeneral wheel
    ( name    = "wheel"
    , n       = 5
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



end OmniWheelVertical;
