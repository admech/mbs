within MbsLite.Test.Omni;

package TestParams

  constant FrictionParams frictionParams =
    TestFrictionParams.impactLikeViscous;

  constant Params pmm = CreateParams
    ( name            =  "params used in the PMM paper"

    , cut             =  true
    , NWheels         =  3
    , nRollers        =  5
    , mecanumAngle    =  0

    , platformRadius  =  0.15
    , wheelRadius     =  0.07071067812

    , platformMass    =  1
    , wheelHubMass    =  0.15
    , rollerMass      =  0.05
    );

  function CalculateVerificationParams
    input Real fractionOfRollerMassInWholeWheel;
    output Params params;
  algorithm
    params := CreateParams
      ( name            =  "params used in the PMM paper"
  
      , cut             =  true
      , NWheels         =  3
      , nRollers        =  5
      , mecanumAngle    =  0
  
      , platformRadius  =  0.15
      , wheelRadius     =  0.05
  
      , platformMass    =  1
      , wheelHubMass    =  0.15
      , fractionOfRollerMassInWholeWheel = fractionOfRollerMassInWholeWheel
      );
  end CalculateVerificationParams;

  constant OmniVehicleParams pmmAtRest = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.atRest(
          TestParams.pmm
        )

    , frictionParams
        = frictionParams

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmSelfRot = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.selfRot(
          TestParams.pmm
        )
    , frictionParams
        = frictionParams

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmStraight = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.straight(
          TestParams.pmm
        )

    , frictionParams
        = frictionParams

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmWrench = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.wrench(
          TestParams.pmm
        )

    , frictionParams
        = frictionParams

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmAtRestDry = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.atRest(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.dryRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmSelfRotDry = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.selfRot(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.dryRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmStraightDry = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.straight(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.dryRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmWrenchDry = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.wrench(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.dryRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmAtRestViscous = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.atRest(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.impactLikeViscous

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmSelfRotViscous = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.selfRot(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.impactLikeViscous

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmStraightViscous = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.straight(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.impactLikeViscous

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmWrenchViscous = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.wrench(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.impactLikeViscous

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmWrenchViscousJointsFrictionSmall = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.wrench(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.impactLikeViscousJointsFrictionSmall

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmWrenchViscousJointsFrictionLarge = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.wrench(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.impactLikeViscousJointsFrictionLarge

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmStraightViscous = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.straight(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.impactLikeViscous

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  // ETA VERIFICATION

  constant OmniVehicleParams pmmSelfRotDryEta1e1 = CalculateOmniVehicleParams
    ( params
        = CalculateVerificationParams
            ( fractionOfRollerMassInWholeWheel = 1e-1
            )
    , initials
        = TestInitials.selfRot(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.dryRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmSelfRotDryEta1e2 = CalculateOmniVehicleParams
    ( params
        = CalculateVerificationParams
            ( fractionOfRollerMassInWholeWheel = 1e-2
            )
    , initials
        = TestInitials.selfRot(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.dryRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmSelfRotDryEta1e3 = CalculateOmniVehicleParams
    ( params
        = CalculateVerificationParams
            ( fractionOfRollerMassInWholeWheel = 1e-3
            )
    , initials
        = TestInitials.selfRot(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.dryRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmSelfRotDryEta1e4 = CalculateOmniVehicleParams
    ( params
        = CalculateVerificationParams
            ( fractionOfRollerMassInWholeWheel = 1e-4
            )
    , initials
        = TestInitials.selfRot(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.dryRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmSelfRotDryEta1e5 = CalculateOmniVehicleParams
    ( params
        = CalculateVerificationParams
            ( fractionOfRollerMassInWholeWheel = 1e-5
            )
    , initials
        = TestInitials.selfRot(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.dryRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmSelfRotDryEta1e6 = CalculateOmniVehicleParams
    ( params
        = CalculateVerificationParams
            ( fractionOfRollerMassInWholeWheel = 1e-6
            )
    , initials
        = TestInitials.selfRot(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.dryRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmStraightDryEta1e1 = CalculateOmniVehicleParams
    ( params
        = CalculateVerificationParams
            ( fractionOfRollerMassInWholeWheel = 1e-1
            )
    , initials
        = TestInitials.straight(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.dryRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmStraightDryEta1e2 = CalculateOmniVehicleParams
    ( params
        = CalculateVerificationParams
            ( fractionOfRollerMassInWholeWheel = 1e-2
            )
    , initials
        = TestInitials.straight(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.dryRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmStraightDryEta1e3 = CalculateOmniVehicleParams
    ( params
        = CalculateVerificationParams
            ( fractionOfRollerMassInWholeWheel = 1e-3
            )
    , initials
        = TestInitials.straight(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.dryRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmStraightDryEta1e4 = CalculateOmniVehicleParams
    ( params
        = CalculateVerificationParams
            ( fractionOfRollerMassInWholeWheel = 1e-4
            )
    , initials
        = TestInitials.straight(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.dryRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmStraightDryEta1e5 = CalculateOmniVehicleParams
    ( params
        = CalculateVerificationParams
            ( fractionOfRollerMassInWholeWheel = 1e-5
            )
    , initials
        = TestInitials.straight(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.dryRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmStraightDryEta1e6 = CalculateOmniVehicleParams
    ( params
        = CalculateVerificationParams
            ( fractionOfRollerMassInWholeWheel = 1e-6
            )
    , initials
        = TestInitials.straight(
          TestParams.pmm
        )

    , frictionParams
        = TestFrictionParams.dryRubberConcrete

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

end TestParams;
