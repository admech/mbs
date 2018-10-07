within MbsLite.Util.Asserts;

function Assert
  input Boolean condition;
  input String message;
  input Boolean silent = false;
algorithm
 
  if not condition then
    assert(false, message);
    Modelica.Utilities.System.exit(-1);
  else
    if not silent then
      print("Assertion passed: " + message);
    end if;
  end if;

end Assert;
