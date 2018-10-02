within MbsLite.Test.Omni;

model OmniWheelVerticalTest

  import MbsLite.Examples.OmniVehicle.PointContact.OmniWheelVertical;

  OmniWheelVertical m;

  annotation(experiment
    ( StopTime = 1
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

end OmniWheelVerticalTest;