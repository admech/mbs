within MbsLite.Examples.OmniVehicle;

package Utils
  
  function CreateParams
    input String   name;
    input Boolean  cut = false;
  
    input Integer  NWheels;
    input Integer  nRollers;
    input Real     mecanumAngle;
  
    input Real     platformRadius;
    input Real     wheelRadius;
  
    input Real     platformMass;
    input Real     wheelHubMass;
    input Real     rollerMass;

    output Params params;

  protected
    Real rollerHalfAngle       = pi / nRollers;
    Real fatRollerHalfAngle    = pi / (if cut then (nRollers - 1) else nRollers);
    Real wheelHubRadius        = wheelRadius * cos(fatRollerHalfAngle);
    Real rollerRadius          = (wheelRadius - wheelHubRadius) / 2;
    Real rollerLength          = wheelRadius * sin(fatRollerHalfAngle);

  algorithm
    params := Params
      ( name            = name
      
      , NWheels         = NWheels
      , nRollers        = nRollers
      , mecanumAngle    = mecanumAngle
      
      , platformRadius  = platformRadius
      , wheelRadius     = wheelRadius
      
      , platformMass    = platformMass
      , wheelHubMass    = wheelHubMass
      , rollerMass      = rollerMass
    
      , rollerHalfAngle       = rollerHalfAngle
      , wheelHubRadius        = wheelHubRadius
      , rollerRadius          = rollerRadius
      , rollerLength          = rollerLength
      
      , platformAxialMoi      = CylinderAxialMoi     ( platformMass,  platformRadius )
      , platformOrthogonalMoi = CylinderOrthogonalMoi( platformMass,  platformRadius, 0.01 )
      , wheelHubAxialMoi      = CylinderAxialMoi     ( wheelHubMass,  wheelHubRadius )
      , wheelHubOrthogonalMoi = CylinderOrthogonalMoi( wheelHubMass,  wheelHubRadius, 0.01 )
      , rollerAxialMoi        = CylinderAxialMoi     ( rollerMass,    rollerRadius   )
      , rollerOrthogonalMoi   = CylinderOrthogonalMoi( rollerMass,    rollerRadius,   rollerLength )
      );

  end CreateParams;

  function CreateInitials
    input String   name;

    input Real     omega;
    input Real     vAbs;
    input Real     vDirAngle;

    output Initials initials;

  algorithm

    initials := Initials
      ( name      = name

      , omega     = omega
      , vAbs      = vAbs
      , vDirAngle = vDirAngle

      , vVec      = vAbs * QToT(QRot(vDirAngle, vertical)) * forward
      , omegaVec  = omega * vertical
      );

  end CreateInitials;

end Utils;
