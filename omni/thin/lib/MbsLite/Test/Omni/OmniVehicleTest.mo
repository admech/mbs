within MbsLite.Test.Omni;

model OmniVehicleTest

  OmniVehicle m
    ( ovp =
        TestParams.pmmWrench
        /*
        TestParams.pmmAtRest
        TestParams.pmmSelfRot
        TestParams.pmmStraight
        TestParams.pmmWrench
        */
    );

  annotation(experiment
    ( StopTime =
        1000
        /*
        20
        0.0001
        */
    , NumberOfIntervals =
        1000000
        /*
        100000
        20000
        10
        5
        */
    , Tolerance = 1e-9
    // , Algorithm = "Dassl"
    ));
equation
  when time == 1 then
    assert
      ( MbsLite.CompareReal(0, 0)
      , "zero should be zero, was: " + String(0)
      );
  end when;

end OmniVehicleTest;
