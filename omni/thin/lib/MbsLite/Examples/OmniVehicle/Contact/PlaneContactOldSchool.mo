within MbsLite.Examples.OmniVehicle.Contact;

model PlaneContactOldSchool
 "A -- plane, B -- roller"
  extends Constraint;

  parameter Params params;

protected
  Real[3] contactPointCoords;
  Real[3] contactPointVelocity;
  Real    cosBetweenRollerVerticalAndGlobalVertical;
  Boolean isInContact;

equation

  cosBetweenRollerVerticalAndGlobalVertical
    = (InPortB.T * vertical) * vertical;

  isInContact
    = cosBetweenRollerVerticalAndGlobalVertical > cos(params.rollerHalfAngle)
      and InPortB.r[2] < params.wheelRadius;
  
  contactPointCoords = if isInContact
      then InPortB.r                                        // start from roller center, 
           + params.wheelHubRadius * (InPortB.T * vertical) // go to wheel center
           - params.wheelRadius * vertical                  // and then outright downward.
      else zeros(3);
  when isInContact <> pre(isInContact) then
    reinit
      ( contactPointCoords
      , if isInContact
        then InPortB.r                                        // start from roller center,
             + params.wheelHubRadius * (InPortB.T * vertical) // go to wheel center
             - params.wheelRadius * vertical                  // and then outright downward.  
        else zeros(3)
      );
  end when;

  contactPointVelocity = Euler
    ( InPortB.r
    , contactPointCoords
    , InPortB.v
    , InPortB.omega
    );

  OutPortA.P = { contactPointCoords[1], contactPointCoords[2], contactPointCoords[3] };

  OutPortB.P = contactPointCoords;
  OutPortB.F = zeros(3);
  OutPortB.M = zeros(3);

end PlaneContactOldSchool;

