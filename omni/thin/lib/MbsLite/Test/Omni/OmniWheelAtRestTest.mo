within MbsLite.Test.Omni;

model OmniWheelAtRestTest

  import MbsLite.Examples.OmniVehicle.Params;
  import MbsLite.Examples.OmniVehicle.Initials;
  import MbsLite.Examples.OmniVehicle.Full.OmniWheelAtRest;

  // NOTE: this field should be a constant becaus OMC can't handle it being a parameter...
  constant Params   params   = TestParams.pmm;
  constant Initials initials = Initials
    ( name = "wheel rotating"
    , omega = 0
    , vAbs = 0
    , vDirAngle = 0
    , vVec = zeros(3)
    , omegaVec = userward + vertical
    );

  OmniWheelAtRest m
    ( Gravity  = zeros(3)
    , nActual  = params.nRollers
    , r0       = params.wheelRadius * vertical
    , q0       = QRot(0, vertical)
    , params   = params
    , initials = initials
    );

  annotation(experiment
    ( StopTime = 100
    , NumberOfIntervals = 10000
    , Tolerance = 1e-9
    , Algorithm = "Dassl"
    ));
equation
  when CompareReal(time, 1) then
    assert
      ( MbsLite.CompareReal(0, 0)
      , "zero should be zero, was: " + String(0)
      );
  end when;

end OmniWheelAtRestTest;
