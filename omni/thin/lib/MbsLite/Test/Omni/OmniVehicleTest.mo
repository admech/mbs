within MbsLite.Test.Omni;

model OmniVehicleTest

  import MbsLite.Examples.OmniVehicle.Params;
  import MbsLite.Examples.OmniVehicle.Initials;
  import MbsLite.Examples.OmniVehicle.Full.OmniVehicle;

  constant Params   params   = TestParams.pmm;
  constant Initials initials
    = TestInitials.atRest;

  OmniVehicle m
    ( gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical)
    , params              = params
    , initials            = initials
    );

  annotation(experiment
    ( StopTime = 1
    , NumberOfIntervals = 100
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

end OmniVehicleTest;
