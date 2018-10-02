within MbsLite.Util.Assert;

function Assert
  input Boolean condition;
  input String message;
algorithm
 
  if not condition then
    assert(false, message);
    Modelica.Utilities.System.exit(-1);
  end if;

end Assert;
