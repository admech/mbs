within MbsLite.Test.Omni;

model RollerOnPlaneExplicitTest

  import MbsLite.Examples.OmniVehicle.Params;
  import MbsLite.Examples.OmniVehicle.Initials;
  import MbsLite.Examples.OmniVehicle.Full.RollerOnPlaneExplicit;

  constant Params   params   = TestParams.pmm;
  constant Initials initials
    // = TestInitials.wheelStill;
    // = TestInitials.wheelSelfRotatingInPlace;
    // = TestInitials.wheelSelfRotatingAndSliding;
    // = TestInitials.wheelSliding;
    = TestInitials.wheelSlidingAlongItsAxis;
    // = TestInitials.wheelRolling_R_0_05;

  RollerOnPlaneExplicit m
    ( Gravity  = -vertical
    , nActual  = params.nRollers
    , r0       = params.wheelRadius * vertical
    , q0       = QRot(0, vertical)
    , params   = params
    , initials = initials
    , coefficientOfFriction        = 1e-1
    , viscousFrictionVelocityBound = 1e-6
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

end RollerOnPlaneExplicitTest;
