#include "rclcpp/rclcpp.hpp"
#include "uav_msgs/srv/zone.hpp"
#include "uav_msgs/msg/zone_array.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "std_msgs/msg/string.hpp"

#include <memory>
#include <cmath>

#include <cstdlib>

class DistributionZone : public rclcpp::Node
{
public:
  DistributionZone()
  : Node("distribution_zone")
  {

    service = this->create_service<uav_msgs::srv::Zone>("zones", std::bind(&DistributionZone::add, this, std::placeholders::_1, std::placeholders::_2));
  }

private:

    rclcpp::Service<uav_msgs::srv::Zone>::SharedPtr service;



    // std::shared_ptr<std::vector<std::vector<float>>> weight_for_all_drone = std::make_shared<std::vector<std::vector<float>>>();

    void add(const std::shared_ptr<uav_msgs::srv::Zone::Request> request, std::shared_ptr<uav_msgs::srv::Zone::Response> response)
    {
      std::shared_ptr<std::vector<geometry_msgs::msg::Polygon>> polygon = std::make_shared<std::vector<geometry_msgs::msg::Polygon>>(); // Множество зон для распределения по дронам
      std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> the_centers_of_the_zones = std::make_shared<std::vector<geometry_msgs::msg::Point32>>(); // Центры зон из множества зон

      std::shared_ptr<std::vector<float>>  the_areas_of_the_polygon_zones = std::make_shared<std::vector<float>>();
      std::shared_ptr<std::vector<nav_msgs::msg::Odometry>> odometry = std::make_shared<std::vector<nav_msgs::msg::Odometry>>(); // Позиции дронов
      // std::shared_ptr<std::vector<nav_msgs::msg::Odometry>> odometry_check = std::make_shared<std::vector<nav_msgs::msg::Odometry>>();
      std::map<std::string, std::string> frame_id_info;

      std::shared_ptr<std::vector<std::vector<geometry_msgs::msg::Polygon>>> polygon_for_all_drone = std::make_shared<std::vector<std::vector<geometry_msgs::msg::Polygon>>>(); // Множество зон для каждого дрона


      //distances_from_the_drone_to_the_center_of_the_zone - массив в массиве должен содержать в себе двумерную матрицу, где каждая строка - расстояния от дрона[0] до центра зон. Аналогично следующие строки только уже дрон[1] и тд

      *polygon = request->zones;
      *odometry = request->odom;

      if (polygon->empty())
      {
      RCLCPP_INFO_STREAM(this->get_logger(), "polygon is empty");
      return;
      }

      if (odometry->empty())
      {
      RCLCPP_INFO_STREAM(this->get_logger(), "polygon is empty");
      return;
      }


      for (int index = 0; index < odometry->size(); ++index)
      {
          
          frame_id_info["frame_id_drone_" + std::to_string(index)] = odometry->at(index).header.frame_id;
          polygon_for_all_drone->emplace_back();

      }

      add_areas_and_centers(polygon, the_centers_of_the_zones, the_areas_of_the_polygon_zones); // заполняем the_areas_of_the_polygon_zones и the_centers_of_the_zones

      while(!polygon->empty())
      {
        for (int drone_number = 0; drone_number < odometry->size(); ++drone_number)
        {
          // if (polygon->empty()) break;

          if (polygon->size() == 1) 
          {

            polygon_for_all_drone->at(drone_number).push_back(polygon->at(0));
            auto iter_polygon = polygon->cbegin();
            polygon->erase(iter_polygon);

            break;
          }
          std::shared_ptr<std::vector<std::vector<float>>> drone_weights = std::make_shared<std::vector<std::vector<float>>>();  // TODO перенести в фор для дронов
          std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> busy_centers_of_the_zones_c_z = std::make_shared<std::vector<geometry_msgs::msg::Point32>>(); // TODO перенести в фор для дронов
          std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> busy_centers_of_the_zones_e_d = std::make_shared<std::vector<geometry_msgs::msg::Point32>>(); // TODO перенести в фор для дронов
          std::shared_ptr<std::vector<float>> distances_from_the_drone_to_the_center_of_the_zone = std::make_shared<std::vector<float>>();  // TODO перенести в фор для дронов но не обязательно
          std::shared_ptr<std::pair<float, float>>  min_max_distance_r = std::make_shared<std::pair<float, float>>(); // TODO перенести в фор для дронов
          std::shared_ptr<std::pair<std::pair<float, float>, float>>  min_max_average_area = std::make_shared<std::pair<std::pair<float, float>, float>>(); // TODO перенести в фор для дронов
          std::shared_ptr<std::pair<float, float>>  min_max_c_z = std::make_shared<std::pair<float, float>>(); // TODO перенести в фор для дронов
          std::shared_ptr<std::pair<float, float>>  min_max_e_d = std::make_shared<std::pair<float, float>>(); // TODO перенести в фор для дронов

          find_max_min_average_area(the_areas_of_the_polygon_zones, polygon, min_max_average_area); // заполняем min_max_average_area

          find_distances_from_the_drone_to_the_center_of_the_zone_and_max_min_distances(drone_number, distances_from_the_drone_to_the_center_of_the_zone, min_max_distance_r, odometry, the_centers_of_the_zones); // заполняем distances_from_the_drone_to_the_center_of_the_zone и min_max_distance_r

          calculate_busy_centers_of_zones(drone_number, polygon_for_all_drone, busy_centers_of_the_zones_c_z , busy_centers_of_the_zones_e_d, odometry->size());

          calculation_of_weights(polygon, drone_number, drone_weights, odometry, the_centers_of_the_zones, min_max_average_area, the_areas_of_the_polygon_zones, min_max_distance_r, distances_from_the_drone_to_the_center_of_the_zone, busy_centers_of_the_zones_c_z, min_max_c_z, busy_centers_of_the_zones_e_d, min_max_e_d ); // заполняем drone_weights

          distribution_of_zones_considering_weights(drone_number, odometry, drone_weights, the_centers_of_the_zones, busy_centers_of_the_zones_c_z, busy_centers_of_the_zones_e_d, polygon, polygon_for_all_drone, the_areas_of_the_polygon_zones); // распределение зон учитывая веса

        }
      }

      for (int drone_number = 0; drone_number < polygon_for_all_drone->size(); ++drone_number)
      {
        RCLCPP_INFO(this->get_logger(), "Зоны дрона %i", drone_number);
        
        uav_msgs::msg::ZoneArray zones_for_drone;
        for (int zone_number = 0; zone_number < polygon_for_all_drone->at(drone_number).size(); ++zone_number)
        {
          RCLCPP_INFO(this->get_logger(), "Zone number %i", zone_number);
          zones_for_drone.zones.push_back(polygon_for_all_drone->at(drone_number).at(zone_number));

          for (int point_number = 0; point_number < zones_for_drone.zones.at(zone_number).points.size(); ++point_number)
          {
              int i = 0;
              
              RCLCPP_INFO(this->get_logger(), "       Point:%i  %f", point_number, zones_for_drone.zones.at(zone_number).points.at(point_number).x);
              RCLCPP_INFO(this->get_logger(), "       Point:%i  %f", point_number, zones_for_drone.zones.at(zone_number).points.at(point_number).y);
              RCLCPP_INFO(this->get_logger(), "       Point:%i  %f", point_number, zones_for_drone.zones.at(zone_number).points.at(point_number).z);
              RCLCPP_INFO_STREAM(this->get_logger(), "");

              ++i;
          }

        }
      
        response->zones_for_drones_array.push_back(zones_for_drone);

      }

      
    }

    void calculate_busy_centers_of_zones(int drone_current, 
    std::shared_ptr<std::vector<std::vector<geometry_msgs::msg::Polygon>>> polygon_for_all_drone, 
    std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> busy_centers_of_the_zones_c_z , 
    std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> busy_centers_of_the_zones_e_d, 
    int drone_count)
    {
      for (int other_drones = 0; other_drones < drone_count; ++other_drones)
      {
          RCLCPP_INFO_STREAM(this->get_logger(), "зашёл в фор занятых центров зон");
        
        if (other_drones != drone_current)
        {
          RCLCPP_INFO_STREAM(this->get_logger(), "зашёл в if занятых центров зон");

          for (int busy_zone_other_drone = 0; busy_zone_other_drone < polygon_for_all_drone->at(other_drones).size(); ++busy_zone_other_drone)
          {
            RCLCPP_INFO_STREAM(this->get_logger(), "зашёл в фор занятых центров зон");

            geometry_msgs::msg::Point32 zone_center = calculate_zone_center(polygon_for_all_drone->at(other_drones).at(busy_zone_other_drone));
            busy_centers_of_the_zones_c_z->push_back(zone_center);
            if (busy_zone_other_drone == (polygon_for_all_drone->at(other_drones).size() - 1))
            {
              RCLCPP_INFO_STREAM(this->get_logger(), "зашёл в if занятых центров зон");

              busy_centers_of_the_zones_e_d->push_back(zone_center);
            }
          }
        }
      }
    }


    geometry_msgs::msg::Point32 calculate_zone_center(geometry_msgs::msg::Polygon zone)
    {
      std::vector<float> x_array;
      std::vector<float> y_array;
      for (int point_zone_number = 0; point_zone_number < zone.points.size(); ++point_zone_number) // добавляем координаты х и у в массивы x_array и y_array
      {

        x_array.push_back(zone.points.at(point_zone_number).x);
        y_array.push_back(zone.points.at(point_zone_number).y);

      }

      #warning сделать проверку размеров массивов x_array == y_array в виде исключения?

      float sum_x_in_array = 0.0;
      float sum_y_in_array = 0.0;

      for (const auto& element_x : x_array) 
      {
        sum_x_in_array += element_x;
      }

      for (const auto& element_y : y_array) 
      {
        sum_y_in_array += element_y;
      }

      geometry_msgs::msg::Point32 polygon_center;
      polygon_center.x = sum_x_in_array / x_array.size(); //координата х центра зоны
      polygon_center.y = sum_y_in_array / y_array.size(); //координата у центра зоны
      return polygon_center;
    }


    void add_areas_and_centers(std::shared_ptr<std::vector<geometry_msgs::msg::Polygon>> polygon, std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> the_centers_of_the_zones, std::shared_ptr<std::vector<float>>  the_areas_of_the_polygon_zones) //находение площадей зон и центров зон
    {
        for (int zone_number = 0; zone_number < polygon->size();++zone_number)
        {

          std::vector<float> x_array;
          std::vector<float> y_array;
          for (int point_zone_number = 0; point_zone_number < polygon->at(zone_number).points.size(); ++point_zone_number) // добавляем координаты х и у в массивы x_array и y_array
          {

            x_array.push_back(polygon->at(zone_number).points.at(point_zone_number).x);
            y_array.push_back(polygon->at(zone_number).points.at(point_zone_number).y);

          }

          #warning сделать проверку размеров массивов x_array == y_array в виде исключения?

          float sum_x_in_array = 0.0;
          float sum_y_in_array = 0.0;

          for (const auto& element_x : x_array) 
          {
            sum_x_in_array += element_x;
          }

          for (const auto& element_y : y_array) 
          {
            sum_y_in_array += element_y;
          }

          geometry_msgs::msg::Point32 polygon_points;
          polygon_points.x = sum_x_in_array / x_array.size(); //координата х центра зоны
          polygon_points.y = sum_y_in_array / y_array.size(); //координата у центра зоны
          the_centers_of_the_zones->push_back(polygon_points); //нахождение центра зоны и добавляние её в массив the_centers_of_the_zones
          RCLCPP_INFO(this->get_logger(), "ЦЕНТР ЗОНЫ %i такой: по х = %f по у = %f", zone_number, polygon_points.x, polygon_points.y);


          float sum_area = 0.0f; // Инициализация переменной для площади

          for (size_t index = 0; index < x_array.size(); ++index)
          {
              size_t next_index = (index + 1) % x_array.size(); // Индекс следующей точки (с циклом)

              // Используем формулу для вычисления площади
              sum_area += polygon->at(zone_number).points.at(index).x * polygon->at(zone_number).points.at(next_index).y;
              sum_area -= polygon->at(zone_number).points.at(next_index).x * polygon->at(zone_number).points.at(index).y;
          }

          float area = std::abs(sum_area) / 2.0f;
          the_areas_of_the_polygon_zones->push_back(area);
          RCLCPP_INFO(this->get_logger(), "area %f", area);
        }
    }


    void find_max_min_average_area(std::shared_ptr<std::vector<float>>  the_areas_of_the_polygon_zones,
                                  std::shared_ptr<std::vector<geometry_msgs::msg::Polygon>> polygon,
                                  std::shared_ptr<std::pair<std::pair<float, float>, float>>  min_max_average_area) // нахождение максимальной, минимальной, средней площади среди зон
    {
        

        float min_area = the_areas_of_the_polygon_zones->at(0);
        float max_area = 0.0;
        float average_area = 0.0;

        for (const auto& area : *the_areas_of_the_polygon_zones)

        // нахождение максимальной и минимальной площади среди зон
        {
          
          if (area > max_area)
          {
            max_area = area;
          }

          if (area < min_area)
          {
            min_area = area;
          }
        }

        if (polygon->size() == 2)
        {
          min_max_average_area->first.first = min_area; // заполнение минимальной, средней и максимальной площади зоны
          min_max_average_area->first.second = max_area;
          return;
        }

        div_t average_of_zones_number = div(the_areas_of_the_polygon_zones->size(), 2); // Нахождение средней площади зоны среди всех зон
        if (the_areas_of_the_polygon_zones->size() % 2 == 0)
        {
          average_area = the_areas_of_the_polygon_zones->at(average_of_zones_number.quot + 1);
        }
        else average_area = the_areas_of_the_polygon_zones->at(average_of_zones_number.quot + average_of_zones_number.rem);

        min_max_average_area->first.first = min_area; // заполнение минимальной, средней и максимальной площади зоны
        min_max_average_area->first.second = max_area;
        min_max_average_area->second = average_area;

        RCLCPP_INFO(this->get_logger(), "минимальная %f средняя %f максимальная %f ", min_max_average_area->first.first, min_max_average_area->first.second, min_max_average_area->second);


    }


    void find_distances_from_the_drone_to_the_center_of_the_zone_and_max_min_distances(int drone_number, std::shared_ptr<std::vector<float>> distances_from_the_drone_to_the_center_of_the_zone, std::shared_ptr<std::pair<float, float>>  min_max_distance_r, std::shared_ptr<std::vector<nav_msgs::msg::Odometry>> odometry, std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> the_centers_of_the_zones) // заполнение массива расстояний от каждого дрона до центра каждой зоны и нахождение мин, макс расстояния от дрона до центра зон
    {

        float x_drone = odometry->at(drone_number).pose.pose.position.x;
        float y_drone = odometry->at(drone_number).pose.pose.position.y;


        for (int number_of_zone = 0; number_of_zone < the_centers_of_the_zones->size(); ++number_of_zone) // нахождение расстояния от дрона до центра зон
        {
          RCLCPP_INFO_STREAM(this->get_logger(), "В форе ");

          float distance = sqrt( pow(x_drone - the_centers_of_the_zones->at(number_of_zone).x, 2) + pow(y_drone - the_centers_of_the_zones->at(number_of_zone).y, 2));
          distances_from_the_drone_to_the_center_of_the_zone->push_back(distance); 
          RCLCPP_INFO(this->get_logger(), "для дрона под номером %i дистанция номер %i такая %f (должно быть %f)", drone_number, number_of_zone, distances_from_the_drone_to_the_center_of_the_zone->at(number_of_zone), distance);

        }
        

        float min_distance = distances_from_the_drone_to_the_center_of_the_zone->at(0);
        float max_distance = 0.0;

        for (int radius_number = 0; radius_number < distances_from_the_drone_to_the_center_of_the_zone->size(); ++radius_number) 
        // нахождение минимального расстояния и максимального расстояния от дрона до центра зон
        {
          // distances_from_the_drone_to_the_center_of_the_zone->at(radius_number); 
          if (distances_from_the_drone_to_the_center_of_the_zone->at(radius_number) > max_distance) 
          {
            max_distance = distances_from_the_drone_to_the_center_of_the_zone->at(radius_number);
          }

          if (min_distance > distances_from_the_drone_to_the_center_of_the_zone->at(radius_number))
          {
            min_distance = distances_from_the_drone_to_the_center_of_the_zone->at(radius_number);
          }
        }

        min_max_distance_r->first = min_distance; //добавление минимума и максимума в std::pair
        min_max_distance_r->second = max_distance; //добавление минимума и максимума в std::pair
        RCLCPP_INFO(this->get_logger(), "минимальная дистанция %f максимальная дистанция %f", min_max_distance_r->first, min_max_distance_r->second);
    }


    void calculation_of_weights(
      std::shared_ptr<std::vector<geometry_msgs::msg::Polygon>> polygon,
      int drone_number,
      std::shared_ptr<std::vector<std::vector<float>>> drone_weights, 
      std::shared_ptr<std::vector<nav_msgs::msg::Odometry>> odometry, 
      std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> the_centers_of_the_zones, 
      std::shared_ptr<std::pair<std::pair<float, float>, float>> min_max_average_area, 
      std::shared_ptr<std::vector<float>> the_areas_of_the_polygon_zones, 
      std::shared_ptr<std::pair<float, float>> min_max_distance_r,
      std::shared_ptr<std::vector<float>> distances_from_the_drone_to_the_center_of_the_zone,
      std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> busy_centers_of_the_zones_c_z,
      std::shared_ptr<std::pair<float, float>> min_max_c_z,
      std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> busy_centers_of_the_zones_e_d,
      std::shared_ptr<std::pair<float, float>> min_max_e_d)
    {


      std::vector<float> c_z_tmp = calculation_of_the_minimum_and_maximum_c_z(busy_centers_of_the_zones_c_z, the_centers_of_the_zones, min_max_c_z);
      std::vector<float> e_d_tmp = calculation_of_the_minimum_and_maximum_e_d(busy_centers_of_the_zones_e_d, the_centers_of_the_zones, min_max_e_d);
      for (const auto c_z : c_z_tmp)
      {
        RCLCPP_ERROR_STREAM(this->get_logger(), "C_Z = " << c_z);

      }
      RCLCPP_ERROR_STREAM(this->get_logger(), "МИНИМАЛЬНЫЙ И МАКСИМАЛЬНЫЙ C_Z" << min_max_c_z->first << min_max_c_z->second);
      RCLCPP_ERROR_STREAM(this->get_logger(), "МИНИМАЛЬНЫЙ И МАКСИМАЛЬНЫЙ e_d" << min_max_e_d->first << min_max_e_d->second);
            
      // RCLCPP_ERROR_STREAM(this->get_logger(), "C_Z = " << c_z_tmp.size());
      // RCLCPP_ERROR_STREAM(this->get_logger(), "e_d = " << e_d_tmp.size());

      if (c_z_tmp.empty())
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "c_z_tmp is empty");

      }
      else RCLCPP_INFO_STREAM(this->get_logger(), "c_z_tmp is NOT empty");

      if (e_d_tmp.empty())
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "e_d_tmp is empty");
      }
      else RCLCPP_INFO_STREAM(this->get_logger(), "e_d_tmp is NOT empty");

      for (int index = 0; index < odometry->size(); ++index)
      {
      drone_weights->emplace_back();
      }

      for (int zone_number = 0; zone_number < the_centers_of_the_zones->size(); ++zone_number) // заполнение массива весов для каждого дрона отдельно
      {

        float b_from_s = b_s(polygon, zone_number, min_max_average_area, the_areas_of_the_polygon_zones);

        float a_from_r = a_r(zone_number, drone_number, min_max_distance_r, distances_from_the_drone_to_the_center_of_the_zone);

        float c_from_z = 0.0;
        if (!busy_centers_of_the_zones_c_z->empty())
        {
          // RCLCPP_ERROR_STREAM(this->get_logger(), "C_Z = " << c_z_tmp.at(0));

          c_from_z = (min_max_c_z->second - c_z_tmp.at(zone_number)) / (min_max_c_z->second - min_max_c_z->first);

          RCLCPP_ERROR_STREAM(this->get_logger(), "C_from_Z = " << c_from_z);

        }
 
        float e_from_d = 0.0;
        if (!busy_centers_of_the_zones_e_d->empty())
        {
          e_from_d = (min_max_e_d->second - e_d_tmp.at(zone_number)) / (min_max_e_d->second - min_max_e_d->first);
          RCLCPP_ERROR_STREAM(this->get_logger(), "E_from_D = " << c_from_z);

        }
        RCLCPP_INFO_STREAM(this->get_logger(), "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");

        RCLCPP_INFO(this->get_logger(), "a_from_r = %f", a_from_r);
        RCLCPP_INFO(this->get_logger(), "b_from_s = %f", b_from_s);
        RCLCPP_INFO_STREAM(this->get_logger(), "C_from_Z = " << c_from_z);
        RCLCPP_INFO_STREAM(this->get_logger(), "E_from_D = " << e_from_d);
        RCLCPP_INFO_STREAM(this->get_logger(), "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");


        float weight = (2*a_from_r + b_from_s - c_from_z - e_from_d) / 2; // weight = (a(r) + b(s)) / 2 - вес 

        drone_weights->at(drone_number).push_back(weight); 
        RCLCPP_INFO(this->get_logger(), "ВЕС НОМЕР %i РАВЕН %f", zone_number, weight);

      }
        RCLCPP_INFO_STREAM(this->get_logger(), "ВЫШЕЛ ИЗ ФОРА СЧЁТА ВЕСОВ НАХУЙ");

    }


    float b_s(std::shared_ptr<std::vector<geometry_msgs::msg::Polygon>> polygon,
              int weight_number, 
              std::shared_ptr<std::pair<std::pair<float, float>, float>>  min_max_average_area, 
              std::shared_ptr<std::vector<float>>  the_areas_of_the_polygon_zones)
    {
      float b_from_s = 0.0; // b(s)

      //min_max_average_area->first.first - минимальная площадь
      //min_max_average_area->first.second - максимальная площадь
      //min_max_average_area->second - средняя площадь
      //the_areas_of_the_polygon_zones и distances_from_the_drone_to_the_center_of_the_zone
      
      float min_ar = min_max_average_area->first.first;
      float max_ar = min_max_average_area->first.second;

      if (polygon->size() == 2)
      {
        b_from_s = (the_areas_of_the_polygon_zones->at(weight_number) - min_ar + 0.0f)/(max_ar - min_ar); // b(s)
        return b_from_s;
      }

      float average_ar = min_max_average_area->second;

      RCLCPP_INFO(this->get_logger(), "min_ar = %f max_ar = %f average_ar = %f the_areas_of_the_polygon_zones->at(weight_number) = %f", min_ar, max_ar, average_ar, the_areas_of_the_polygon_zones->at(weight_number));


      if (the_areas_of_the_polygon_zones->at(weight_number) < average_ar)
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "ЗАШЁЛ В ПЕРВЫЙ ИФ ДЛЯ B_FROM_S");

        b_from_s = (the_areas_of_the_polygon_zones->at(weight_number) - min_ar + 0.0f)/(average_ar - min_ar); // b(s)
        RCLCPP_INFO(this->get_logger(), "b_from_s В ПЕРВОМ ИФЕ = %f", b_from_s);

      } 
      else
      {
        RCLCPP_INFO_STREAM(this->get_logger(), "ЗАШЁЛ ВО ВТОРОЙ ИФ ДЛЯ B_FROM_S");

        b_from_s = (max_ar - the_areas_of_the_polygon_zones->at(weight_number) + 0.0f)/(max_ar - average_ar); // b(s)
      }
      
      RCLCPP_INFO(this->get_logger(), "B_FROM_S = %f", b_from_s);
      
      return b_from_s;
    }


    float a_r(int weight_number, int drone_number, std::shared_ptr<std::pair<float, float>>  min_max_distance_r, std::shared_ptr<std::vector<float>> distances_from_the_drone_to_the_center_of_the_zone)
    {

      RCLCPP_INFO(this->get_logger(), "количество дронов %i номер веса %i", drone_number, weight_number);

      float min_dis = min_max_distance_r->first; // минимальное расстояние от дрона до одного из центров зон
      float max_dis = min_max_distance_r->second; // максимальное расстояние от дрона до одного из центров зон
      RCLCPP_INFO(this->get_logger(), "МИН МАКС ДИСТАНЦИИ РАБОТАЮТ %f %f", min_dis, max_dis);

      float a_from_r = 0.0; // b(s)
      a_from_r = (max_dis - distances_from_the_drone_to_the_center_of_the_zone->at(weight_number))/(max_dis - min_dis); // a(r)

      return a_from_r;
    }


    std::vector<float> calculation_of_the_minimum_and_maximum_c_z(std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> busy_centers_of_the_zones_c_z, std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> the_centers_of_the_zones, std::shared_ptr<std::pair<float, float>>  min_max_c_z)
    {
      std::vector<float> c_z;
      if (!busy_centers_of_the_zones_c_z->empty())
      {
        for (int zone_candidate_number = 0; zone_candidate_number < the_centers_of_the_zones->size(); ++zone_candidate_number) 
        {
          float distance_c_z;
          for (int busy_zone_number = 0; busy_zone_number < busy_centers_of_the_zones_c_z->size(); ++busy_zone_number)
          {

            float busy_x = busy_centers_of_the_zones_c_z->at(busy_zone_number).x;
            float busy_y = busy_centers_of_the_zones_c_z->at(busy_zone_number).y;

            distance_c_z += sqrt( pow(busy_x - the_centers_of_the_zones->at(zone_candidate_number).x, 2) + pow(busy_y - the_centers_of_the_zones->at(zone_candidate_number).y, 2));
            
          }

            float c_z_tmp = busy_centers_of_the_zones_c_z->size() / distance_c_z;
            c_z.push_back(c_z_tmp);
            // RCLCPP_ERROR_STREAM(this->get_logger(), "C_Z и номер индекса" << zone_candidate_number);
            // RCLCPP_ERROR_STREAM(this->get_logger(), "C_Z и номер индекса" << c_z.at(0));

        }
        // RCLCPP_ERROR_STREAM(this->get_logger(), "C_Z = " << c_z.at(0));
        // RCLCPP_ERROR_STREAM(this->get_logger(), "C_Z = " << c_z.size());

        float c_z_min = c_z.at(0);
        float c_z_max = 0;

        for (int c_z_number = 0; c_z_number < c_z.size(); ++c_z_number)
        {
          if (c_z.at(c_z_number) > c_z_max) c_z_max = c_z.at(c_z_number);
          if (c_z.at(c_z_number) < c_z_min) c_z_min = c_z.at(c_z_number);
        }

        min_max_c_z->first = c_z_min;
        min_max_c_z->second = c_z_max;
      
        return c_z;

      }

      return c_z;
    }



    std::vector<float> calculation_of_the_minimum_and_maximum_e_d(std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> busy_centers_of_the_zones_e_d, std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> the_centers_of_the_zones, std::shared_ptr<std::pair<float, float>>  min_max_e_d)
    {
      std::vector<float> e_d;
      if (!busy_centers_of_the_zones_e_d->empty())
      {
        for (int zone_number = 0; zone_number < the_centers_of_the_zones->size(); ++zone_number) 
        {
          float distance_e_d;
          for (int busy_zone_number = 0; busy_zone_number < busy_centers_of_the_zones_e_d->size(); ++busy_zone_number)
          {

            float busy_x = busy_centers_of_the_zones_e_d->at(busy_zone_number).x;
            float busy_y = busy_centers_of_the_zones_e_d->at(busy_zone_number).y;

            distance_e_d += sqrt( pow(busy_x - the_centers_of_the_zones->at(zone_number).x, 2) + pow(busy_y - the_centers_of_the_zones->at(zone_number).y, 2));
            
          }

            float e_d_cal = busy_centers_of_the_zones_e_d->size() / distance_e_d;
            e_d.push_back(e_d_cal);

        }

        float e_d_min = e_d.at(0);
        float e_d_max = 0;

        for (int e_d_number = 0; e_d_number < e_d.size(); ++e_d_number)
        {
          if (e_d.at(e_d_number) > e_d_max) e_d_max = e_d.at(e_d_number);
          if (e_d.at(e_d_number) < e_d_min) e_d_min = e_d.at(e_d_number);
        }

        min_max_e_d->first = e_d_min;
        min_max_e_d->second = e_d_max;
        
        return e_d;

      }
      return e_d;
    }


    void distribution_of_zones_considering_weights(
      int drone_number,
      std::shared_ptr<std::vector<nav_msgs::msg::Odometry>> odometry, 
      std::shared_ptr<std::vector<std::vector<float>>> drone_weights, 
      std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> the_centers_of_the_zones, 
      std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> busy_centers_of_the_zones_c_z, 
      std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> busy_centers_of_the_zones_e_d, 
      std::shared_ptr<std::vector<geometry_msgs::msg::Polygon>> polygon,
      std::shared_ptr<std::vector<std::vector<geometry_msgs::msg::Polygon>>> polygon_for_all_drone, 
      std::shared_ptr<std::vector<float>> the_areas_of_the_polygon_zones) // распределение зон учитывая веса
    {
      // Найти зону с наибольшим количеством очков, добавить к текущему дрону и удалить из всех необходимых массивов (удалить из списка незанятых зон)

      float max_weight = 0.0;
      int number_max_weight = 0;
      for (int zone_index = 0; zone_index < the_areas_of_the_polygon_zones->size(); ++zone_index)
      {
        if (drone_weights->at(drone_number).at(zone_index) > max_weight)
        {
          max_weight = drone_weights->at(drone_number).at(zone_index);
          number_max_weight = zone_index;
        }   
      }
      
      RCLCPP_INFO(this->get_logger(), "максимальный вес %f номер веса %i", max_weight, number_max_weight);

      polygon_for_all_drone->at(drone_number).push_back(polygon->at(number_max_weight));

      double new_x_drone = static_cast<double>(the_centers_of_the_zones->at(number_max_weight).x);
      double new_y_drone = static_cast<double>(the_centers_of_the_zones->at(number_max_weight).y);

      RCLCPP_INFO(this->get_logger(), "СТАРАЯ ПОЗИЦИЯ ДРОНА Х У %f %f", odometry->at(drone_number).pose.pose.position.x, odometry->at(drone_number).pose.pose.position.y);

      odometry->at(drone_number).pose.pose.position.x = new_x_drone;
      odometry->at(drone_number).pose.pose.position.y = new_y_drone;

      RCLCPP_INFO(this->get_logger(), "НОВАЯ ПОЗИЦИЯ ДРОНА Х У %f %f", odometry->at(drone_number).pose.pose.position.x, odometry->at(drone_number).pose.pose.position.y);

      RCLCPP_INFO(this->get_logger(), "КОЛИЧЕСТВО ЗОН БЫЛО %i", polygon->size());

      auto iter_polygon = polygon->cbegin();
      polygon->erase(iter_polygon + number_max_weight);

      RCLCPP_INFO(this->get_logger(), "КОЛИЧЕСТВО ЗОН СТАЛО %i", polygon->size());

      RCLCPP_INFO(this->get_logger(), "КОЛИЧЕСТВО ПЛОЩАДЕЙ БЫЛО %i", the_areas_of_the_polygon_zones->size());

      auto iter_areas = the_areas_of_the_polygon_zones->cbegin();
      the_areas_of_the_polygon_zones->erase(iter_areas + number_max_weight);

      RCLCPP_INFO(this->get_logger(), "КОЛИЧЕСТВО ПЛОЩАДЕЙ СТАЛО %i", the_areas_of_the_polygon_zones->size());

      RCLCPP_INFO(this->get_logger(), "КОЛИЧЕСТВО ЦЕНТРОВ БЫЛО %i", the_centers_of_the_zones->size());

      auto iter_centers = the_centers_of_the_zones->cbegin();
      the_centers_of_the_zones->erase(iter_centers + number_max_weight);

      RCLCPP_INFO(this->get_logger(), "КОЛИЧЕСТВО ЦЕНТРОВ СТАЛО %i", the_centers_of_the_zones->size());

    }
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistributionZone>());
  rclcpp::shutdown();
  return 0;
}




    // float c_z(
    //   int zone_number, 
    //   std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> busy_centers_of_the_zones_c_z, 
    //   std::shared_ptr<std::pair<float, float>>  min_max_c_z, 
    //   std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> the_centers_of_the_zones)
    // {
    //   if (!busy_centers_of_the_zones_c_z->empty())
    //   {

    //     float min_c_z = min_max_c_z->first; 
    //     float max_c_z = min_max_c_z->second;

    //     float distance_c_z;
    //     for(int busy_zone_number = 0; busy_zone_number < busy_centers_of_the_zones_c_z->size(); ++busy_zone_number)
    //     {

    //       float busy_x = busy_centers_of_the_zones_c_z->at(busy_zone_number).x;
    //       float busy_y = busy_centers_of_the_zones_c_z->at(busy_zone_number).y;

    //       distance_c_z += sqrt( pow(busy_x - the_centers_of_the_zones->at(zone_number).x, 2) + pow(busy_y - the_centers_of_the_zones->at(zone_number).y, 2));

    //     }
    //     float c_from_z = 0.0;
    //     distance_c_z = busy_centers_of_the_zones_c_z->size()/distance_c_z;
    //     c_from_z = (max_c_z - distance_c_z)/(max_c_z - min_c_z);
    //     return c_from_z;

    //   }

    //   else
    //   {
    //     float c_from_z = 0.0;
    //     return c_from_z;
    //   }
      
    // }




    // float e_d(
    //   int zone_number, 
    //   int drone_number, 
    //   std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> busy_centers_of_the_zones_e_d, 
    //   std::shared_ptr<std::pair<float, float>>  min_max_e_d, 
    //   std::shared_ptr<std::vector<geometry_msgs::msg::Point32>> the_centers_of_the_zones)
    // {
    //   if (!busy_centers_of_the_zones_e_d->empty())
    //   {

    //     float min_e_d = min_max_e_d->first; 
    //     float max_e_d = min_max_e_d->second;

    //     float distance_e_d;
    //     for(int busy_zone_number = 0; busy_zone_number < busy_centers_of_the_zones_e_d->size(); ++busy_zone_number)
    //     {
    //       float busy_x = busy_centers_of_the_zones_e_d->at(busy_zone_number).x;
    //       float busy_y = busy_centers_of_the_zones_e_d->at(busy_zone_number).y;

    //       distance_e_d += sqrt( pow(busy_x - the_centers_of_the_zones->at(zone_number).x, 2) + pow(busy_y - the_centers_of_the_zones->at(zone_number).y, 2));
    //     }
    //     float e_from_d = 0.0;
    //     distance_e_d = busy_centers_of_the_zones_e_d->size()/distance_e_d;
    //     e_from_d = (max_e_d - distance_e_d)/(max_e_d - min_e_d);
    //     return e_from_d;

    //   }

    //   else
    //   {
    //     float e_from_d = 0.0;
    //     return e_from_d;
    //   }
       
    // }

