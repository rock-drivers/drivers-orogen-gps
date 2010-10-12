
require 'widget_grid'
require 'orocos'
Orocos.initialize

widget_grid = WidgetGrid.new

Orocos.run 'test_gpsd' do
  gpsd = Orocos::TaskContext.get 'Task'
  gpsd.hostename ='fritsche'
  gpsd.configure
  gpsd.start
  widget_grid.display gpsd
  widget_grid.run
end

