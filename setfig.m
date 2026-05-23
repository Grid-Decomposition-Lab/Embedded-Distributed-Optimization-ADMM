function setfig(LineWidth,FontSize,MarkerSize)
% setfig sets line width, font size and marker size of a figure
%
% syntax: setfig or setfig(LineWidth,FontSize,MarkerSize)
%
% input:  LineWidth   width of the lines in the figure
%         FontSize    size of the font in the figure
%         MarkerSize  size of the marker in the figure
%
    
% 2015 Feb 10, Henrik Gert Hassager

if nargin<1,
  LineWidth=1.5;
  FontSize=14;
  MarkerSize=floor(FontSize/2);
end

AxisBoxLineWidth=LineWidth;
AxisPlotLineWidth=LineWidth;
AxisTextFontSize=FontSize;
AxisLabelFontSize=FontSize;
AxisMarkerSize=MarkerSize;

LegendObjectLineWidth=LineWidth;
LegendBoxLineWidth=LineWidth;
LegendTextFontSize=FontSize;
LegendMarkerSize=MarkerSize;

hfig=gcf;
hfc=get(hfig,'Children');
for i=1:length(hfc),
  
  if strcmp(char(get(hfc(i),'Tag')),'legend'),
    % Legend
    set(hfc(i),'LineWidth',LegendBoxLineWidth);
    set(hfc(i),'FontSize',LegendTextFontSize); 
    
    hlc=get(hfc(i),'Children');
    for j=1:length(hlc)
      if strcmp(char(get(hlc(j),'Type')),'line'),
        set(hlc(j),'LineWidth',LegendObjectLineWidth);
        set(hlc(j),'MarkerSize',LegendMarkerSize);
      else
        set(hlc(j),'FontUnits','points');
        set(hlc(j),'FontSize',LegendTextFontSize);
      end
    end

  else
    % Plot
    haxis=hfc(i);

    xlim=get(haxis,'XLim');
    ylim=get(haxis,'YLim');
    
    set(haxis,'FontSize',AxisTextFontSize);
    set(haxis,'LineWidth',AxisBoxLineWidth);
    set(haxis,'Box','On');
    
    
    hxlabel=get(haxis,'XLabel');
    hylabel=get(haxis,'YLabel');
    hzlabel=get(haxis,'ZLabel');
    titlelabel=get(haxis,'Title');
    
    set(hxlabel,'FontSize',AxisLabelFontSize);
    set(hylabel,'FontSize',AxisLabelFontSize);
    set(hylabel,'FontSize',AxisLabelFontSize);
    set(titlelabel,'FontSize',AxisLabelFontSize);
    
    hac=get(haxis,'Children');
    for j=1:length(hac)
      set(hac(j),'LineWidth',AxisPlotLineWidth);
      set(hac(j),'MarkerSize',AxisMarkerSize);
    end
    
    set(haxis,'XLim',xlim);
    set(haxis,'YLim',ylim);    
    
  end
end

hold off
drawnow
