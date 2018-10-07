within MbsLite.Test.Omni;

model OmniWheelOnPlaneTest

  import MbsLite.Examples.OmniVehicle.Params;
  import MbsLite.Examples.OmniVehicle.Initials;
  import MbsLite.Examples.OmniVehicle.Full.OmniWheelOnPlaneFree;

  constant Params   params   = TestParams.pmm;
  constant Initials initials = Initials
    ( name = "wheel rolling"
    , omega = 0 // -params.wheelRadius * 1
    , vAbs = 0 // 1
    , vDirAngle = 0
    , vVec = zeros(3) // 1 * forward
    , omegaVec = zeros(3) // -params.wheelRadius * 1 * userward
    );

  OmniWheelOnPlaneFree m
    ( Gravity  = zeros(3)
    , nActual  = params.nRollers
    , r0       = params.wheelRadius * vertical
    , q0       = QRot(0, vertical)
    , params   = params
    , initials = initials
    );

  annotation(experiment
    ( StopTime = 10
    , NumberOfIntervals = 1000
    , Tolerance = 1e-9
    , Algorithm = "Dassl"
    ));
equation
  when time == 1 then
    assert
      ( MbsLite.CompareReal(0, 0)
      , "zero should be zero, was: " + String(0)
      );
  end when;

end OmniWheelOnPlaneTest;
