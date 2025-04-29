function i = integration(vector1,time,c)

arguments
    vector1 (:,1) double
    time (:,1) double
    c double
  end
  
  i = [0; cumsum(vector1.*diff(time))] + c;


end