within MbsLite.Test.Omni;

package TestParams
  import MbsLite.Examples.OmniVehicle.Params;
  import MbsLite.Examples.OmniVehicle.Initials;
  import MbsLite.Examples.OmniVehicle.OmniVehicleParams;
  import MbsLite.Examples.OmniVehicle.CalculateOmniVehicleParams;

  constant Params pmm = CreateParams
    ( name            =  "params used in the PMM paper"

    , cut             =  true
    , NWheels         =  3
    , nRollers        =  5
    , mecanumAngle    =  0

    , platformRadius  =  0.15
    , wheelRadius     =  0.05

    , platformMass    =  1
    , wheelHubMass    =  0.15
    , rollerMass      =  0.05
    );

  constant OmniVehicleParams pmmAtRest = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.atRest

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmSelfRot = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.selfRot

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmStraight = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.straight

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

  constant OmniVehicleParams pmmWrench = CalculateOmniVehicleParams
    ( params
        = TestParams.pmm
    , initials
        = TestInitials.wrench

    , gravity             = -vertical
    , platformQuaternion  = QRot(0, vertical) 
    );

end TestParams;
