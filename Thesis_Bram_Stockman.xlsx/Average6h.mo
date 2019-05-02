within GIT_thesis_Bram_Stockman;
model Average6h "Calculate the running mean temperature of 6 hours"

   discrete Modelica.Blocks.Interfaces.RealOutput TRm(unit="K",displayUnit = "degC")
    "Running mean average temperature"
     annotation (Placement(transformation(extent={{96,-10},{116,10}})));
    Modelica.Blocks.Sources.RealExpression TAmb(y=Input_average)
    annotation (Placement(transformation(extent={{-74,-10},{-54,10}})));
    Modelica.Blocks.Interfaces.RealInput Input_average
    "Connector of setpoint input signal"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
protected
  parameter Modelica.SIunits.Time t_start(fixed=false) "Start time of the model";
  parameter Real coeTRm[6] = {1, 0.8, 0.6, 0.5, 0.4, 0.3}./3.6
    "weighTAmb.yg coefficient for the running average";
  discrete Real[6] TAveHour(each unit="K",each displayUnit = "degC")
    "Vector with the average hour temperatures of the previous nTermRm hours";
  Real intTAmb "integral of TAmb.y";

initial equation
  intTAmb=0;
  t_start = time;
  TAveHour=ones(6).*Input_average;
  TRm=Input_average;
equation
  der(intTAmb) =  TAmb.y;
algorithm
  when sample(t_start+3600,3600) then
    // Update of TAveHour
    for i in 2:6 loop
      TAveHour[i] := pre(TAveHour[i-1]);
    end for;
    TAveHour[1] := intTAmb /3600;
    TRm :=TAveHour*coeTRm;
  end when;

equation
    // reinitialisation of the intTAmb
  when sample(t_start+3600,3600) then
    reinit(intTAmb,0);
  end when;
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}})),
    experiment(StopTime=864000),
    __Dymola_experimentSetupOutput,
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
        graphics={
        Rectangle(
          extent={{100,100},{-100,-100}},
          lineColor={100,100,100},
          fillPattern=FillPattern.Solid,
          fillColor={255,255,255}),
        Line(
          points={{0,100},{98,0},{0,-100}},
          color={100,100,100},
          smooth=Smooth.None),
        Text(
          extent={{-100,140},{100,100}},
          lineColor={0,0,255},
          textString="%name"),
        Text(
          extent={{-48,32},{58,-26}},
          lineColor={0,0,255},
          textString="6 hours")}),
Documentation(revisions="<html>
<ul>
<li>
April 17, 2018, by Damien Picard:<br/>
Add t_start in sample to compute correctly for non zero initial time.<br/>
Use Input_average as initialization instead of an arbitrary value of 283.15K.
</li>
<li>
January 19, 2015, by Damien Picard:<br/>
First implementation.
</li>
</ul>
</html>"));
end Average6h;
