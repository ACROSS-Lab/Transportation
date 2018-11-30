/**
* Name: raster13
* Author: dongh
* Description: time fixed
* Tags: Tag1, Tag2, TagN
*/
model final_1h_real

/* Insert your model definition here */
global
{
	file boundary <- file("../includes2/boundary.shp");
	file road_shape <- file("../includes2/road.shp");
	file line <- file("../includes2/movingline.shp");
	file in_point <- file("../includes2/in.shp");
	file out_point <- file("../includes2/out.shp");
	file wait_point <- file("../includes2/wait.shp");
	file light <- file("../includes2/light.shp");
	graph road_network;
	// Hình ảnh
	file Motor <- file("../images/motor2.jpg");
	file Bus <- file("../images/bus.jpg");
	file Car <- file("../images/car.jpg");
	geometry shape <- envelope(boundary);
	// các dữ liệu global cần lấy
	int total_waiting_time <- 0;
	int total_bus_in <- 0;
	int total_car_in <- 0;
	int total_motor_in <- 0;
	init
	{
		create bound from: boundary;
		create road from: road_shape;
		create lane from: line;
		loop i from: 1 to: 5
		{
			create intersection with: [id::i];
		}

		create lights from: light with: [direction::int(read('id')), intersect::int(read('intersect'))];
		create in_pt from: in_point {
			direction <- int(read('id'));
			intersect <- int(read('intersect')); 
			motor_ratio <- float(read('mratio')); 
			car_ratio <- float(read('cratio')); 
			bus_ratio <- float(read('bratio'));
			number_of_flows <- int(read('nof'));
			add float(read('flow_low')) to: flows;
			add float(read('flow_med')) to: flows;
			add float(read('flow_hig')) to: flows;
			flow <- flows[0];
		}
		create out_pt from: out_point with: [direction::int(read('id')), intersect::int(read('intersect'))];
		create wait_pt from: wait_point with: [direction::int(read('direction')), intersect::int(read('intersect'))];
		road_network <- as_edge_graph(lane);
	}
	//end simulation
	reflex end when: cycle = 43200001
	{
		do pause;
	}

	// save file
	reflex save when: mod(cycle, 180000) = 1
	{
		//save (total_waiting_time) to: '../data_change_flow/total_waiting.txt' type: text rewrite: false;
		save (total_bus_in) to: '../data_change_flow/bus.txt' type: text rewrite: false;
		save (total_car_in) to: '../data_change_flow/car.txt' type: text rewrite: false;
		save (total_motor_in) to: '../data_change_flow/motor.txt' type: text rewrite: false;
	}
	// sinh phương tiện
	// 1s = 50 cycle => 1 min = 3000 cycle
	reflex born
	{
		loop i over: in_pt
		{
			int p <- min([1, poisson(i.flow / 3000)]);
			if (p > 0)
			{
				float type_vehicles <- rnd(999) / 10;
				if (type_vehicles < i.motor_ratio)
				{
					create motor number: p
					{
						total_motor_in <- total_motor_in + p;
						obstacle_species <- [motor, car, bus];
						start <- i;
						living_space <- 1000.0;
						tolerance <- 0.1;
						lanes_attribute <- "nbLanes";
						location <- point(start);
					}

				} else if (type_vehicles < i.motor_ratio + i.car_ratio)
				{
					create car number: p
					{
						total_car_in <- total_car_in + p;
						obstacle_species <- [motor, car, bus];
						start <- i;
						living_space <- 1500.0;
						tolerance <- 0.1;
						lanes_attribute <- "nbLanes";
						location <- point(start);
					}

				} else
				{
					create bus number: p
					{
						total_bus_in <- total_bus_in + p;
						obstacle_species <- [motor, car, bus];
						start <- i;
						living_space <- 2000.0;
						tolerance <- 0.1;
						lanes_attribute <- "nbLanes";
						location <- point(start);
					}
				}
			}
		}
	}
}

species bound
{
	aspect default
	{
		draw shape color: # lightgray;
	}

}

species road
{
	aspect geom
	{
		draw (shape + 50) color: # black;
	}

}

species in_pt{
	int direction;
	int intersect;
	int number_of_flows;
	list<float> flows;
	float flow;
	float motor_ratio;
	float car_ratio;
	float bus_ratio;
	list<int> time_changes <- [1, 117, 135, 162, 288, 324, 360, 414, 0];
	int period <- 0;
	int old_time_change;
	int time_change <- time_changes[period] * 10000;
	reflex change_flow  {
		int real_cycle2 <- mod(cycle, 4320000);
		if(real_cycle2 = time_change ){
			period <- mod(period, 8) + 1;
			old_time_change <- cycle;
			time_change <- time_changes[period] * 10000;
		}
		if(period = 1){
			flow <- flows[0];
		}
		else if(period = 2){
			flow <- flows[0] + (flows[2] - flows[0])*(cycle - old_time_change)/180000;
		}
		else if(period = 3){
			flow <- flows[2] - ((flows[2] - flows[1])*(cycle - old_time_change)/180000)/1.5;
		}
		else if(period = 4){
			flow <- flows[1];
		}
		else if(period = 5){
			flow <- flows[1] + ((flows[2] - flows[1])*(cycle - old_time_change)/180000)/2;
		}
		else if(period = 6){
			flow <- flows[2] - ((flows[2] - flows[1])*(cycle - old_time_change)/180000)/2;
		}
		else if(period = 7){
			flow <- flows[1] - ((flows[1] - flows[0])*(cycle - old_time_change)/180000)/3;
		}
		else if(period = 8){
			flow <- flows[0];
		}
	}
}

species out_pt
{
	int direction;
	int intersect;
}

species wait_pt
{
	int intersect;
	int direction;
}

species lane
{
	aspect default
	{
		draw shape color: # violet;
	}

}

species vehicles skills: [driving]
{
	in_pt start <- nil;
	out_pt target <- nil;
	lights light <- nil;
	float base_speed;
	float speed;
	int stop_time <- 0;
	int current_intersect <- 0;
	int current_direction;
	int next_direction;
	wait_pt current_wpt;
	// các dữ liệu vehicle cần lấy
	int waiting_time <- 0;
	int riding_time <- 0;
	bool is_counted_stop <- false;
	int waiting_number <- 0;
	
	bool stop <- false;
	int postion <- -1;
	int weight;

	// xác định tham số cho phương tiện
	reflex come_in when: target = nil
	{
		if (target = nil)
		{
			if (start.intersect < 3)
			{
				target <- one_of(out_pt where (each.intersect != start.intersect or each.direction != start.direction));
			} else
			{
				target <- one_of(out_pt where (each.intersect > start.intersect or (each.intersect = start.intersect and each.direction != start.direction)));
			}

		}

		if (start.intersect > 0)
		{
			current_intersect <- start.intersect;
			current_direction <- start.direction;
		} else
		{
			current_intersect <- int(signum(target.intersect - 2.5) * 0.5 + 2.5);
			current_direction <- int(signum(target.intersect - 2.5) * 0.5 + 1.5);
		}

		if (current_intersect = target.intersect)
		{
			next_direction <- target.direction;
		} else
		{
			next_direction <- int(1.5 - signum(target.intersect - current_intersect) * 0.5);
		}

		light <- lights first_with (each.intersect = current_intersect and each.direction = current_direction * 10 + next_direction);
		current_wpt <- wait_pt first_with (each.intersect = current_intersect and each.direction = current_direction);
	}

	// nhận biết đèn giao thông
	reflex determine when: current_wpt = nil
	{
		if (target.intersect - current_intersect = 0)
		{
			current_wpt <- wait_pt first_with (each.intersect = current_intersect and each.direction = current_direction);
		} else if (abs(current_intersect - target.intersect) = 1)
		{
			current_intersect <- current_intersect + signum(target.intersect - current_intersect) * 1;
			current_direction <- 3 - next_direction;
			next_direction <- target.direction;
			light <- lights first_with (each.intersect = current_intersect and each.direction = current_direction * 10 + next_direction);
			current_wpt <- wait_pt first_with (each.intersect = current_intersect and each.direction = current_direction);
		} else
		{
			current_intersect <- current_intersect + signum(target.intersect - current_intersect) * 1;
			current_direction <- 3 - next_direction;
			next_direction <- int(1.5 - signum(target.intersect - current_intersect) * 0.5);
			light <- lights first_with (each.intersect = current_intersect and each.direction = current_direction * 10 + next_direction);
			current_wpt <- wait_pt first_with (each.intersect = current_intersect and each.direction = current_direction);
		}

	}

	//chờ
		reflex wait when: current_wpt != nil and self distance_to current_wpt < speed 
	{
		if (length(bus where (each.light = light and stop = true)) + length(car where (each.light = light and stop = true)) + length(motor where (each.light = light and
		stop = true)) > 0 and light.current_color = #green)
		{
			do stop;
		}

		if (light = nil)
		{
			do turn_right;
		} else if (light.current_color != #green)
		{
			do stop;
		} else if (light.current_color = #green and stop = true)
		{
			do stop;
		} else if (light.current_color = #green and stop = false)
		{
			do move_on_green;
		}

	}
	//cac action
	action turn_right
	{
		current_wpt <- nil;
		intersection intersect <- intersection first_with (each.id = self.current_intersect);
		intersect.number_of_vehicles_pass <- intersect.number_of_vehicles_pass + 1;
		postion <- -1;
	}

	action move_on_green
	{
		current_wpt <- nil;
		speed <- base_speed;
		intersection intersect <- intersection first_with (each.id = self.current_intersect);
		intersect.number_of_vehicles_pass <- intersect.number_of_vehicles_pass + 1;
		if(stop = true){intersect.intersect_waiting_time <- intersect.intersect_waiting_time + cycle - stop_time;}
		is_counted_stop <- false;
		stop <- false;
		postion <- -1;
	}

	action stop
	{
		speed <- 0 # km / # h;
		if (is_counted_stop = false)
		{
			waiting_number <- waiting_number + 1;
			is_counted_stop <- true;
		}

		stop_time <- cycle;
		stop <- true;
		if (postion = -1)
		{
			do take_postion;
		}
	}
	
	action take_postion
	{
		intersection intersect <- intersection first_with (each.id = self.current_intersect);
		if (intersect.current_vehicle_postion[self.current_direction - 1] = 0)
		{
			intersect.current_vehicle_postion[self.current_direction - 1] <- 1;
			self.postion <- intersect.current_vehicle_postion[self.current_direction - 1];
			intersect.current_weight[self.current_direction - 1] <- self.weight;
		} else
		{
			if (self.weight + intersect.current_weight[self.current_direction - 1] <= intersect.capacity[self.current_direction - 1])
			{
				self.postion <- intersect.current_vehicle_postion[self.current_direction - 1];
				intersect.current_weight[self.current_direction - 1] <- intersect.current_weight[self.current_direction - 1] + self.weight;
			} else
			{
				intersect.current_vehicle_postion[self.current_direction - 1] <- intersect.current_vehicle_postion[self.current_direction - 1] + 1;
				self.postion <- intersect.current_vehicle_postion[self.current_direction - 1];
				intersect.current_weight[self.current_direction - 1] <- self.weight;
			}

		}

	}

	reflex rising_speed when: speed < base_speed and stop != true
	{
		speed <- speed + base_speed / 150;
	}

	//di chuyển
	reflex move
	{
		do goto target: target on: road_network speed: speed * (1 + rnd(-0.1, 0.1));
		riding_time <- riding_time + 1;
	}

	//biến mất khi đến đích
	reflex disappear when: location = point(target)
	{
		//total_waiting_time <- total_waiting_time + waiting_time;
		//save (riding_time) to: '../data_change_flow/riding_time.txt' type: text rewrite: false;
		save (waiting_number) to: '../data_change_flow/waiting_number.txt' type: text rewrite: false;
		do die;
	}

	aspect default
	{
		draw circle(2000) color: # red;
	}

}

species motor parent: vehicles
{
	float base_speed <- rnd(150, 200) # km / # h;
	float speed <- base_speed;
	int weight <- 1;
	aspect new
	{
		draw Motor size: { 1500, 800 } rotate: heading;
	}

}

species car parent: vehicles
{
	float base_speed <- rnd(250, 320) # km / # h;
	float speed <- base_speed;
	int weight <- 4;
	aspect new
	{
		draw Car size: { 2500, 1500 } rotate: heading;
	}

}

species bus parent: vehicles
{
	float base_speed <- rnd(230, 280) # km / # h;
	float speed <- base_speed;
	int weight <- 10;
	aspect new
	{
		draw Bus size: { 3500, 2200 } rotate: heading;
	}

}

species intersection
{
	int id;
	int temp;
	intersection neighbor;
	// lưu lượng xe tối đa thông qua ngã tư theo các hướng 1, 2, 3, 4
	list<int> capacity <- [20, 20, 20, 20];
	list<int> current_vehicle_postion <- [0, 0, 0, 0];
	list<int> current_weight <- [0, 0, 0, 0];
	list<int> time_of_phases; //list thời gian cho các phase
	int current_phase <- 1; // phase hiện tại
	int phase_time;
	int yt <- 3; //yellow state time 
	int time <- 0;
	// các dữ liệu cần lấy
	int intersect_waiting_time <- 0;
	int number_of_vehicles_pass <- 0;
	init
	{
		if (self.id = 1)
		{
			time_of_phases <- [30, 30, 15, 15];
			yt <- 5;
		} else if (self.id = 2)
		{
			time_of_phases <- [30, 30];
		} else if (self.id = 3)
		{
			time_of_phases <- [50, 30];
		} else if (self.id = 4)
		{
			time_of_phases <- [57, 26];
		} else if (self.id = 5)
		{
			time_of_phases <- [60, 30];
		}

		phase_time <- time_of_phases[current_phase - 1];
	}

	reflex count_time when: mod(cycle, 50) = 0
	{
		time <- time + 1;
	}

	reflex change_phase when: time = phase_time
	{
		if (self.id = 1)
		{
			current_phase <- mod(current_phase, 4) + 1;
		} else
		{
			current_phase <- mod(current_phase, 2) + 1;
		}

		phase_time <- time_of_phases[current_phase - 1];
		time <- 0;
	}

	reflex save_intersect when: mod(cycle, 180000) = 1
	{
		save (intersect_waiting_time) to: '../data_change_flow/intersect_waiting_time' + self.id + '.txt' type: text rewrite: false;
		save (number_of_vehicles_pass) to: '../data_change_flow/intersect_pass' + self.id + '.txt' type: text rewrite: false;
	}

	int count_motor_stop (intersection inter, int direction)
	{
		return motor count (each.current_intersect = inter.id and (each.current_direction * 10 + each.next_direction) = direction and each.speed < 5 # km / # h);
	}

	int count_car_stop (intersection inter, int direction)
	{
		return car count (each.current_intersect = inter.id and (each.current_direction * 10 + each.next_direction) = direction and each.speed < 5 # km / # h);
	}

	int count_bus_stop (intersection inter, int direction)
	{
		return bus count (each.current_intersect = inter.id and (each.current_direction * 10 + each.next_direction) = direction and each.speed < 5 # km / # h);
	}

	int count_vehicles_stop (intersection inter, int direction)
	{
		return count_motor_stop(inter, direction) + 4 * count_car_stop(inter, direction) + 10 * count_bus_stop(inter, direction);
	}

	int count_motor_stop_time(intersection inter) {
		return (motor where(each.current_intersect = inter.id and each.stop = true)) sum_of((cycle - each.stop_time));
	}
	int count_car_stop_time(intersection inter) {
		return (car where(each.current_intersect = inter.id and each.stop = true)) sum_of((cycle - each.stop_time));
	}
	int count_bus_stop_time(intersection inter) {
		return (bus where(each.current_intersect = inter.id and each.stop = true)) sum_of((cycle - each.stop_time));
	}

}

species lights
{
	rgb current_color <- # red;
	int direction;
	int intersect;
	int queue_length;
	int yellow_count_time <- 0;
	int count_for_green <- 0;
	bool green_counted <- false;
	intersection inter <- intersection first_with (each.id = self.intersect);
	int direct1;
	int direct2;
	reflex change_color when: mod(cycle, 50) = 0
	{
		if (self.intersect = 1)
		{
			switch inter.current_phase
			{
				match 1
				{
					direct1 <- 12;
					direct2 <- 21;
				}

				match 2
				{
					direct1 <- 34;
					direct2 <- 43;
				}

				match 3
				{
					direct1 <- 13;
					direct2 <- 24;
				}

				match 4
				{
					direct1 <- 32;
					direct2 <- 41;
				}

			}

			if (self.direction = direct1 or self.direction = direct2)
			{
				if (count_for_green < 3 and green_counted = false)
				{
					count_for_green <- count_for_green + 1;
					if(count_for_green = 3){ do compute_queue_length;}
				} else
				{
					self.current_color <- # green;
					count_for_green <- 0;
					green_counted <- true;
				}

			} else
			{
				green_counted <- false;
				if (self.current_color = # green)
				{
					self.current_color <- # yellow;
				} else if (self.current_color = # yellow)
				{
					if (yellow_count_time < 3)
					{
						yellow_count_time <- yellow_count_time + 1;
					} else
					{
						self.current_color <- # red;
						yellow_count_time <- 0;
					}

				} else
				{
					self.current_color <- # red;
				}

			}

		} else
		{
			if (inter.current_phase = 1)
			{
				if (self.direction = 12 or self.direction = 21 or self.direction = 13 or self.direction = 24)
				{
					if (count_for_green < 3 and green_counted = false)
					{
						count_for_green <- count_for_green + 1;
						if(count_for_green = 3){ do compute_queue_length;}
					} else
					{
						self.current_color <- # green;
						count_for_green <- 0;
						green_counted <- true;
					}

				} else
				{
					green_counted <- false;
					if (self.current_color = # green)
					{
						self.current_color <- # yellow;
					} else if (self.current_color = # yellow)
					{
						if (yellow_count_time < 3)
						{
							yellow_count_time <- yellow_count_time + 1;
						} else
						{
							self.current_color <- # red;
							yellow_count_time <- 0;
						}

					} else
					{
						self.current_color <- # red;
					}

				}

			} else if (inter.current_phase = 2)
			{
				if (self.direction = 34 or self.direction = 43 or self.direction = 32 or self.direction = 41)
				{
					if (count_for_green < 3 and green_counted = false)
					{
						count_for_green <- count_for_green + 1;
						if(count_for_green = 3){ do compute_queue_length;}
					} else
					{
						self.current_color <- # green;
						count_for_green <- 0;
						green_counted <- true;
					}

				} else
				{
					green_counted <- false;
					if (self.current_color = # green)
					{
						self.current_color <- # yellow;
					} else if (self.current_color = # yellow)
					{
						if (yellow_count_time < 3)
						{
							yellow_count_time <- yellow_count_time + 1;
						} else
						{
							self.current_color <- # red;
							yellow_count_time <- 0;
						}

					} else
					{
						self.current_color <- # red;
					}

				}

			}

		}

	}

	action compute_queue_length{
		queue_length <- inter.count_motor_stop(inter, direction)
						+ 4*inter.count_car_stop(inter, direction)
						+ 10*inter.count_bus_stop(inter, direction);
		save(queue_length) to: '../data_change_flow/max_queue_length_' + inter.id + '_' + self.direction + '.txt' type: text rewrite: false;
		save(queue_length) to: '../data_change_flow/max_queue_length' + div(cycle, 4320000) + '.txt' type: text rewrite: false;
	}
//	action compute_queue_length{
//		queue_length <- inter.count_motor_stop(inter, direction)
//						+ 4*inter.count_car_stop(inter, direction)
//						+ 10*inter.count_bus_stop(inter, direction);
//		save(queue_length) to: '../data_change_flow/max_queue_length' + div(cycle, 180000) + '.txt' type: text rewrite: false;
//	}
	//một giây giải phóng xe một lần
	reflex control_flow when: self.current_color = # green and mod(cycle, 50) = 0
	{
		ask bus where (each.light = self and each.stop = true)
		{
			self.postion <- self.postion - 1;
			if (self.postion = 0)
			{
				do move_on_green;
			}

		}

		ask car where (each.light = self and each.stop = true)
		{
			self.postion <- self.postion - 1;
			if (self.postion = 0)
			{
				do move_on_green;
			}

		}

		ask motor where (each.light = self and each.stop = true)
		{
			self.postion <- self.postion - 1;
			if (self.postion = 0)
			{
				do move_on_green;
			}

		}

		if (inter.current_vehicle_postion[div(direction, 10) - 1] > 0)
		{
			inter.current_vehicle_postion[div(direction, 10) - 1] <- inter.current_vehicle_postion[div(direction, 10) - 1] - 1;
		} else
		{
			inter.current_weight[div(direction, 10) - 1] <- 0;
		}

	}

	aspect default
	{
		draw circle(600) color: current_color;
	}

}

experiment finaldayreal type: gui
{
	output
	{
		display carte type: opengl
		{
			species bound aspect: default;
			species road aspect: geom;
			species motor aspect: new;
			species car aspect: new;
			species bus aspect: new;
			species lights;
			species vehicles;
		}

	}

}
