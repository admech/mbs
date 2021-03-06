within MbsLite.Test.Omni;

package TestInitials

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
    , vDirAngle = 0 // 1e-3
    );

  constant Initials wheelStill = Initials
    ( name = "wheel still"
    , omega = 0
    , vAbs = 0
    , vDirAngle = 0
    , vVec = zeros(3)
    , omegaVec = zeros(3)
    );

  constant Initials wheelSpinning = Initials
    ( name = "wheel still"
    , omega = inf
    , vAbs = 0
    , vDirAngle = 0
    , vVec = zeros(3)
    , omegaVec = 1 * vertical
    );

  constant Initials wheelSelfRotatingInPlace = Initials
    ( name = "wheel rotating CW looking from user"
    , omega = -1
    , vAbs = 0
    , vDirAngle = 0
    , vVec = zeros(3)
    , omegaVec = -1 * userward
    );

  constant Initials wheelSliding = Initials
    ( name = "wheel sliding forward"
    , omega = 0
    , vAbs = 1
    , vDirAngle = 0
    , vVec = 1 * forward
    , omegaVec = zeros(3)
    );

  constant Initials wheelSlidingAlongItsAxis = Initials
    ( name = "wheel sliding forward"
    , omega = 0
    , vAbs = 1
    , vDirAngle = pi / 2
    , vVec = 1 * QToT(QRot(pi / 2, vertical)) * forward
    , omegaVec = zeros(3)
    );

  constant Initials wheelSelfRotatingAndSliding = Initials
    ( name = "wheel rotating CW looking from user and sliding forward"
    , omega = -1
    , vAbs = 1
    , vDirAngle = 0
    , vVec = 1 * forward
    , omegaVec = -1 * userward
    );

  constant Initials wheelRolling_R_0_05 = Initials
    ( name = "wheel rotating CW looking from user and sliding forward without slip"
    , omega = -1
    , vAbs = 0.05
    , vDirAngle = 0
    , vVec = 0.05 * forward
    , omegaVec = -1 * userward
    );

end TestInitials;
