within MbsLite.Test.Omni;

model RollerOnPlaneTest

  import MbsLite.Examples.OmniVehicle.Params;
  import MbsLite.Examples.OmniVehicle.Initials;
  import MbsLite.Examples.OmniVehicle.Full.RollerOnPlane;

  constant Params   params   = TestParams.pmm;
  constant Initials initials =
    TestInitials.wheelSliding;
    /*
    TestInitials.wheelStill;
    TestInitials.wheelSpinning;
    TestInitials.wheelSelfRotatingInPlace;
    TestInitials.wheelSelfRotatingAndSliding;
    TestInitials.wheelSliding;
    TestInitials.wheelSlidingAlongItsAxis;
    TestInitials.wheelRolling_R_0_05;
    */

  RollerOnPlane m
    ( Gravity  = -vertical
    , nActual  = params.nRollers
    , r0       = params.wheelRadius * vertical
    , q0       = QRot(0, -userward)
    , params   = params
    , initials = initials
    );

  annotation(experiment
    ( StopTime = 2
    , NumberOfIntervals = 2000
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

end RollerOnPlaneTest;
