within MbsLite.Test.Omni;

package TestInitials
  import MbsLite.Examples.OmniVehicle.Initials;

  constant Initials atRest = CreateInitials
    ( name      = "atRest"
    , omega     = 0
    , vAbs      = 0
    , vDirAngle = 0
    );

  constant Initials selfRot = CreateInitials
    ( name      = "selfRot"
    , omega     = 1
    , vAbs      = 0
    , vDirAngle = 0
    );

  constant Initials straight = CreateInitials
    ( name      = "straight"
    , omega     = 0
    , vAbs      = 1
    , vDirAngle = 0
    );

  constant Initials wrench = CreateInitials
    ( name      = "wrench"
    , omega     = 1
    , vAbs      = 1
    , vDirAngle = 0
    );

end TestInitials;
