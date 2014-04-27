def monitor_gps(task)
    p = task.process
    
    solution_reader      = task.solution.reader
    constellation_reader = task.constellation.reader
    pos_reader = task.position_samples.reader

    STDERR.puts "Type | PosX PosY PosZ | dLat dLong dAlt | Used usedMinSNR PDOP Tracked trackedMinSNR Known"
    while true
        solution = solution_reader.read
        constellation = constellation_reader.read
        pos = pos_reader.read

        if p && !p.alive?
            raise "module crashed"
        end

        info = "%s " % [task.state.to_s]
        if pos
            info << " | %2.2f %2.2f %2.2f | " % [
                pos.position.x,
                pos.position.y,
                pos.position.z]
        end
        if solution
            info << "%s | %2.2f %2.2f %2.2f | %i " % [
                solution.positionType.to_s,
                solution.deviationLatitude, solution.deviationLongitude,
                solution.deviationAltitude, solution.noOfSatellites]
        end
        if constellation && !constellation.quality.usedSatellites.empty?
            used    = constellation.quality.usedSatellites.
                map do |prn|
                    constellation.satellites.knownSatellites.find { |s| s.PRN == prn }
                end
            tracked = constellation.satellites.knownSatellites.find_all { |s| s.SNR && s.SNR > 0 }
            info << " %2.2f %2.2f %i %2.2f %i" % [used.map { |s| s.SNR }.min,
                constellation.quality.pdop,
                tracked.length, tracked.map { |s| s.SNR }.min,
                constellation.satellites.knownSatellites.length]
        end
        STDERR.puts info
        sleep 1
    end
end

