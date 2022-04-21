void InflationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
 {
   boost::unique_lock < boost::recursive_mutex > lock(*inflation_access_);
   if (!enabled_ || (cell_inflation_radius_ == 0))
     return;
 
   // make sure the inflation list is empty at the beginning of the cycle (should always be true)
   ROS_ASSERT_MSG(inflation_cells_.empty(), "The inflation list must be empty at the beginning of inflation");
 
   unsigned char* master_array = master_grid.getCharMap();
   unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();
 
   if (seen_ == NULL) {
     ROS_WARN("InflationLayer::updateCosts(): seen_ array is NULL");
     seen_size_ = size_x * size_y;
     seen_ = new bool[seen_size_];
   }
   else if (seen_size_ != size_x * size_y)
   {
     ROS_WARN("InflationLayer::updateCosts(): seen_ array size is wrong");
     delete[] seen_;
     seen_size_ = size_x * size_y;
     seen_ = new bool[seen_size_];
   }
   memset(seen_, false, size_x * size_y * sizeof(bool));
 
   // We need to include in the inflation cells outside the bounding
   // box min_i...max_j, by the amount cell_inflation_radius_.  Cells
   // up to that distance outside the box can still influence the costs
   // stored in cells inside the box.
   min_i -= cell_inflation_radius_;
   min_j -= cell_inflation_radius_;
   max_i += cell_inflation_radius_;
   max_j += cell_inflation_radius_;
 
   min_i = std::max(0, min_i);
   min_j = std::max(0, min_j);
   max_i = std::min(int(size_x), max_i);
   max_j = std::min(int(size_y), max_j);
 
   // Inflation list; we append cells to visit in a list associated with its distance to the nearest obstacle
   // We use a map<distance, list> to emulate the priority queue used before, with a notable performance boost
 
   // Start with lethal obstacles: by definition distance is 0.0
   std::vector<CellData>& obs_bin = inflation_cells_[0.0];
   for (int j = min_j; j < max_j; j++)
   {
     for (int i = min_i; i < max_i; i++)
     {
       int index = master_grid.getIndex(i, j);
       unsigned char cost = master_array[index];
       if (cost == LETHAL_OBSTACLE)
       {
         obs_bin.push_back(CellData(index, i, j, i, j));
       }
     }
   }
 
   // Process cells by increasing distance; new cells are appended to the corresponding distance bin, so they
   // can overtake previously inserted but farther away cells
   std::map<double, std::vector<CellData> >::iterator bin;
   for (bin = inflation_cells_.begin(); bin != inflation_cells_.end(); ++bin)
   {
     for (int i = 0; i < bin->second.size(); ++i)
     {
       // process all cells at distance dist_bin.first
       const CellData& cell = bin->second[i];
 
       unsigned int index = cell.index_;
 
       // ignore if already visited
       if (seen_[index])
       {
         continue;
       }
 
       seen_[index] = true;
 
       unsigned int mx = cell.x_;
       unsigned int my = cell.y_;
       unsigned int sx = cell.src_x_;
       unsigned int sy = cell.src_y_;
 
       // assign the cost associated with the distance from an obstacle to the cell
       unsigned char cost = costLookup(mx, my, sx, sy);
       unsigned char old_cost = master_array[index];
       if (old_cost == NO_INFORMATION && cost >= INSCRIBED_INFLATED_OBSTACLE)
         master_array[index] = cost;
       else
         master_array[index] = std::max(old_cost, cost);
 
       // attempt to put the neighbors of the current cell onto the inflation list
       if (mx > 0)
         enqueue(index - 1, mx - 1, my, sx, sy);
       if (my > 0)
         enqueue(index - size_x, mx, my - 1, sx, sy);
       if (mx < size_x - 1)
         enqueue(index + 1, mx + 1, my, sx, sy);
       if (my < size_y - 1)
         enqueue(index + size_x, mx, my + 1, sx, sy);
     }
   }
 
   inflation_cells_.clear();
 }