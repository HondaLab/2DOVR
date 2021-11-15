def write_data(t,dt,l,c,r,timer):
    write_data = []
    write_data.append(t)
    write_data.append(dt)
    write_data.append(l)
    write_data.append(c)
    write_data.append(r)
    write_data.append(timer)
    csv_writer1.writerow(write_data)

        wt = time.time()
        write_data(wt,dt,distanceL,distanceC,distanceR,wtimer)
        wtimer=0
