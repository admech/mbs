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

  constant Initials wheelStill = Initials
    ( name = "wheel still"
    , omega = 0
    , vAbs = 0
    , vDirAngle = 0
    , vVec = zeros(3)
    , omegaVec = zeros(3)
    );

  constant Initials wheelRolling = Initials
    ( name = "wheel rolling"
    , omega = -params.wheelRadius * 1
    , vAbs = 1
    , vDirAngle = 0
    , vVec = 1 * forward
    , omegaVec = -params.wheelRadius * 1 * userward
    );

end TestInitials;
