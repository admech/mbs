within MbsLite.Test.Omni;

model OmniVehicleTest

  import MbsLite.Examples.OmniVehicle.Params;
  import MbsLite.Examples.OmniVehicle.CalculateOmniVehicleParams;
  import MbsLite.Examples.OmniVehicle.Full.OmniVehicle;

  OmniVehicle m
    ( ovp = TestParams.pmmAtRest
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
