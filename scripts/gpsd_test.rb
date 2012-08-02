
#require 'widget_grid'
require 'orocos'
Orocos.initialize

#widget_grid = WidgetGrid.new

Orocos.run 'test_gpsd' do
  gpsd = Orocos::TaskContext.get 'Task'
  gpsd.hostname ='localhost'



#  origin = '/base/Position'
#     origin.x = 64000
#     origin.y = 0
#     origin.z = 0
  
#  gpsd.origin = origin

  gpsd.configure
  gpsd.start
#  widget_grid.display gpsd
#  widget_grid.run

loop do
  sleep 0.1
end

end

