#include "DataProcessControl.h"

DataProcessControl::DataProcessControl(std::vector<std::vector<VdynePointCloud::Point3D>>& Single_Point_Cloud,CTracker* trker)
{

    SinglePointCloud = Single_Point_Cloud;
    tracker = trker;
    init_value  = 0;
    num_of_col = 4;
    dist_min = 100;
    clusterpc.clear();
}



void DataProcessControl::GroundSegmentation(double& ma,double& mb,size_t& reducesize) {


    /// Point Cloud Format
    ///****************************************
    ///  1..... 32
    ///  2..... 32
    ///  .
    ///  .
    ///  .
    ///  n..... 32
    /// ****************************************
    vector<bucket> buckets;
    int row = SinglePointCloud.size();
    int col = SinglePointCloud[0].size();
    ///int n_point = row*col;
    std::vector<Point2f> xzpoints;
    std::vector<double> values;
    //qDebug("row is %d, col is %d", row,col);


    // 700 grids
    std::vector<VdynePointCloud> grid(700);
    std::vector<Point2f> xypoints;
    for(size_t i=0;i<row;i++){
        for(size_t j=0; j<1;j++ ){
           VdynePointCloud::Point3D p;
           p =  SinglePointCloud[i][j];
           if(( abs(p.mY) <1 && p.mX< 1.5 &&p.mX>-3))
           {
              continue;
           }
           double r = sqrt(p.mX*p.mX + p.mY*p.mY);
           if(r<1||r>4){
               continue;
           }
           ///qDebug("dist is %f",r);
           int ind_cir = floor((r-1)/0.3)*70;
           ///qDebug("indvir is %d",ind_cir);
           int ind_rot = floor((p.mRotation*70)/(2*PI));
           ///qDebug("indrot is %d",ind_rot);
           int ind = ind_cir + ind_rot;
           ///qDebug("ind is %d",ind);
           grid[ind].insertPoint(p);
        }
    }



    ///if(max>0.8){
      double max =0.8;
    ///}
   /// if(min<-0.15){
      double min =-0.15;
   /// }

      int inter = 15;
      //create buckets
      double interval = (max-min)/inter;
      for (int i = 1; i <= inter; i++)
      {
          bucket temp = { min+(interval)*(i-1) , min+(interval)*(i), 0};
          buckets.push_back(temp);
      }

      int val_size = xzpoints.size();
      //populate the probability density function
      double weight = 1./(double)val_size;
      std::vector<Point2f> regvalue;



      for(size_t i =0;i<buckets.size();i++)
      {
          double ma = buckets[i].maximum;
          double mi = buckets[i].minimum;
          double fe = buckets[i].frequency;
          double total = 0;
          std::vector<Point2f> temp;
          for(size_t j=0;j<val_size;j++)
          {
              if (xzpoints[j].y<ma && xzpoints[j].y>=mi)
                 {
                   ///std::cout<< "inc   " << j<<std::endl;
                   buckets[i].frequency += weight;
                   total += xzpoints[j].y;
                   temp.push_back(xzpoints[j]);
                 }
          }
          //qDebug("size of temp is %d",temp.size() );
          if(buckets[i].frequency>0.1)
          {
             regvalue.insert(std::end(regvalue), std::begin(temp), std::end(temp));

          }
      }



     LinearRegression lr;
      lr.Init(regvalue);
      lr.Run();
      //lr.PrintResult();

      ma = lr.getMa();
      mb = lr.getMb();
      double A = ma;
      double B = -1;
      double C = mb;
      loss.clear();
      input_data.clear();
      cloud.pts.clear();


      for(size_t i=0;i<row;i++)
      {
          for(size_t j=0; j<col;j++ )
          {
             VdynePointCloud::Point3D pt;
             pt =  SinglePointCloud[i][j];

             if((abs(pt.mY) <1 && pt.mX< 1.5 &&pt.mX>-3))
             {
               G_PointCloud.insertPoint(pt);
               continue;
             }

             if(abs(pt.mX)>20 || abs(pt.mY)>5)
             {
               G_PointCloud.insertPoint(pt);
               continue;
             }

             double x0= pt.mX;
             double y0= pt.mZ;
             double dist = abs(A*x0+B*y0+C)/sqrt(A*A+B*B);
             if(dist>0.2)
             {
               std::vector<double>  dim_data;
               dim_data.push_back(pt.mX);
               dim_data.push_back(pt.mY);
               dim_data.push_back(pt.mZ);
               dim_data.push_back(pt.mIntensity);
               PointCloudType<double>::kPoint pts;
               pts.x = pt.mX;
               pts.y = pt.mY;
               cloud.pts.push_back(pts);
               input_data.push_back(dim_data);
               //in_x.push_back(pt.mX);
               //in_y.push_back(pt.mY);
               //in_z.push_back(pt.mZ);
               //in_intensity.push_back(pt.mIntensity);
               Ng_PointCloud.insertPoint(pt);
             }
             else{
                 G_PointCloud.insertPoint(pt);
                 if(j%2==0 && j<8)
                 loss.push_back(dist);
             }
          }
      }

      reducesize = Ng_PointCloud.getSize();
      num_of_row = reducesize;
      //qDebug("size is %d",reducesize);
      //qDebug("size is %d and %d",input_data.size(),input_data[0].size());
/*
      input_data.resize( num_of_row , std::vector<double>( num_of_col,init_value ));
      for(size_t i = 0;i<num_of_row;i++)
      {
          input_data[i][0] = in_x[i];
          input_data[i][1] = in_y[i];
          input_data[i][2] = in_z[i];
          input_data[i][3] = in_intensity[i];
      }
*/
}


VdynePointCloud DataProcessControl::getSegmentationPointCloud(){

  return Ng_PointCloud;

}


VdynePointCloud DataProcessControl::getGroundPointCloud(){

  return G_PointCloud;

}



void DataProcessControl::AssignCluster(double radius)
{
    int pts_size = num_of_row;
    /*(
    PointCloudType<double> cloud;
    cloud.pts.resize(pts_size);
    for(unsigned int i=0; i<pts_size; i++){
       cloud.pts[i].x = input_data[i][0];
       cloud.pts[i].y = input_data[i][1];
       //cloud.pts[i].z = input_data[i][2];
    }
    */

    // construct a kd-tree index:
    typedef KDTreeSingleIndexAdaptor<
        L2_Simple_Adaptor<double, PointCloudType<double> > ,
        PointCloudType<double>,
        2 /* dim */
        > my_kd_tree_t;
    my_kd_tree_t   index(2 /*dim*/, cloud, KDTreeSingleIndexAdaptorParams(15 /* max leaf */) );
    index.buildIndex();
#if 0
    // Test resize of dataset and rebuild of index:
    cloud.pts.resize(cloud.pts.size()*0.5);
    index.buildIndex();
#endif


    const double search_radius = static_cast<double>(radius);

    clusterLabels.resize(cloud.pts.size(),-1);

    int ClusterID = 0;
    std::vector<std::pair<size_t,double> >   ret_matches;
    std::array<int,70000> idxsInRange;

    //#pragma omp parallel for
    for(size_t pointID=0; pointID<pts_size;++pointID)
    {
        if(clusterLabels[pointID]!=-1)
        {
            continue;
        }

        ret_matches.clear();
        const double query_pt[2] = { input_data[pointID][0], input_data[pointID][1]};
        nanoflann::SearchParams params;
        const size_t nMatches = index.radiusSearch(&query_pt[0],search_radius, ret_matches, params);

        for (size_t i=0;i<nMatches;i++){
             idxsInRange[i] = ret_matches[i].first;
        }

        /// Label the point the same as its first neighbor with a label
        for (size_t j=0;j < nMatches; j++)
        {
           if(clusterLabels[idxsInRange[j]]!=-1)
             {
                 clusterLabels[pointID] = clusterLabels[idxsInRange[j]];
                 break;
             }
        }

        /// If no neighbors already had labels, assign this point to a new cluster
        if(clusterLabels[pointID]==-1)
              {
                clusterLabels[pointID]=ClusterID;
              }

        /// Label all neighbors that are unlabeled the same as the current point
             for (size_t j=0;j < nMatches; j++)
              {
                if(clusterLabels[idxsInRange[j]]==-1)
                {
                   clusterLabels[idxsInRange[j]]=clusterLabels[pointID];
                }
              }

              std::set<int> neighborClusters;
              neighborClusters.insert(clusterLabels[pointID]);
              for(size_t j=0;j < nMatches; j++)
              {
                 neighborClusters.insert(clusterLabels[idxsInRange[j]]);
              }
              Replace(clusterLabels,neighborClusters,clusterLabels[pointID]);
              ClusterID++;
          }
}


void DataProcessControl::Replace(vector<int> &input,set<int> toReplace,int replacement)
{

       for(unsigned int i=0;i<input.size();i++)
       {
          for(set<int>::iterator it1 = toReplace.begin();it1 != toReplace.end();it1++)
          {  if(input[i]==*it1)
             {
                 input[i]=replacement;
             }
          }
       }
}

void DataProcessControl::Replace(vector<int> &input,int toReplace,int replacement)
{
        for(unsigned int i=0;i<input.size();i++)
        {
            if(input[i] == toReplace)
            {
               input[i] = replacement;
            }

        }

}

vector<int> DataProcessControl::ReNumber(vector<int> input)
{
       set<int> usedNumbers;
       for(unsigned int i=0;i<input.size();i++)
       {
          usedNumbers.insert(input[i]);
       }

       int count=1;
       for(set<int>::iterator it1 = usedNumbers.begin();it1 != usedNumbers.end();it1++)
       {
           Replace(input,*it1,count);
           count++;
       }
       return input;

}


void DataProcessControl::BoundedBox(double MaxLength,double MinLength,double MaxWidth,double MinWidth){

     ///qDebug("clsuter size is %d",clusterLabels.size());


     for(unsigned int j=0;j<clusterLabels.size();j++)
     {
        ClusterNumbers.insert(clusterLabels[j]);
     }
     //int num=0;
     for(set<int>::iterator it =  ClusterNumbers.begin();it !=  ClusterNumbers.end();it++)
     {
        //num++;
        //qDebug("cluster num is %d",num);

        vector<double> tempX,tempY,tempZ,tempI,ind;

        for(unsigned int i=0;i<clusterLabels.size();i++)
        {
            if(clusterLabels[i]==*it)
           {
               //ind.push_back(it1-clusterLabels.begin());
               tempX.push_back(input_data[i][0]);
               tempY.push_back(input_data[i][1]);
               tempZ.push_back(input_data[i][2]);
               tempI.push_back(input_data[i][3]);
               ind.push_back(i);
           }

        }

        size_t clustersize = ind.size();

        if (ind.empty()||clustersize<50){
            continue;
        }

       //std::cout << input_data[1][1] <<std::endl;

        vector<double>::const_iterator xmax, ymax,zmax,xmin,ymin,zmin;

        xmax = max_element(std::begin(tempX), std::end(tempX));
        ymax = max_element(std::begin(tempY), std::end(tempY));
        zmax = max_element(std::begin(tempZ), std::end(tempZ));
        xmin = min_element(std::begin(tempX), std::end(tempX));
        ymin = min_element(std::begin(tempY), std::end(tempY));
        zmin = min_element(std::begin(tempZ), std::end(tempZ));

        double dx,dy,dz,xmean,ymean;

        dx = abs(*xmax-*xmin);
        dy = abs(*ymax-*ymin);
        dz = abs(*zmax-*zmin);
        ///qDebug("dx is %f, dy is %f, dz is %f",dx,dy,dz);

/// time conmsuming loops
/// .........................................................................................

        if(dy>MaxLength || dy<MinLength){
            continue;
        }
        else if (dx>MaxWidth||dx<MinWidth){
            continue;
        }
        else if (dz<0.25){
            continue;
        }
///..........................................................................................

       xmean = (*xmax+*xmin)/2;
       ymean = (*ymax+*ymin)/2;

//************************************************************************

       if(xmean<0&&ymean<0){
           xmean = xmean - ((4-dx)/2);
           ymean = ymean - ((2-dy)/2);
       }

       else if(xmean>0&&ymean<0){
           xmean = xmean + ((4-dx)/2);
           ymean = ymean - ((2-dy)/2);

       }

       else if(xmean<0&&ymean>0){
           xmean = xmean - ((4-dx)/2);
           ymean = ymean + ((2-dy)/2);

       }

       else if(xmean>0&&ymean>0){

           xmean = xmean + ((4-dx)/2);
           ymean = ymean + ((2-dy)/2);
       }

//*************************************************************************


       double dist = sqrt(xmean*xmean + ymean*ymean);
       if(dist<dist_min){
          dist_min = dist;
       }


       CenterPtsX.push_back(xmean);
       CenterPtsY.push_back(ymean);
       DeltaX.push_back(dx);
       DeltaY.push_back(dy);
       distance.push_back(dist);
       //qDebug("center is %f  %f \n",xmean,ymean);

       //ClusterPointClound.insertPoint();
       VdynePointCloud::Point3D clusterpoint;
       VdynePointCloud tempcluster;


       for(size_t n=0;n<clustersize;n++){
           clusterpoint.mX = tempX[n];
           clusterpoint.mY = tempY[n];
           clusterpoint.mZ = tempZ[n];
           ClusterPointCloud.insertPoint(clusterpoint);
           tempcluster.insertPoint(clusterpoint);
       }

       clusterpc.push_back(tempcluster);

      //*******************************************************************************
       auto maxI_it = max_element(std::begin(tempI), std::end(tempI));
       int maxI = *maxI_it;

       //Calculate the mean of intensity
       double meanI;
       double sum = 0.0;
       int tempI_size = tempI.size();
       for ( int i=0; i < tempI_size; i++)
       {
           sum += tempI[i];
       }

       meanI = sum / tempI_size;

       // calculate variance of intensity
       double varI;
       double var_temp = 0;
       for(int i = 0; i < tempI_size; i++)
          {
            var_temp += (tempI[i] - meanI) * (tempI[i] - meanI) ;
          }
       varI =  var_temp / tempI_size;

       feat.max_intensity.push_back(maxI);
       feat.mean_intensity.push_back(meanI);
       feat.var_intenisty.push_back(varI);
       feat.width.push_back(dx);
       feat.length.push_back(dy);
       feat.height.push_back(dz);
     }

}

void DataProcessControl::Tracking(){
    //pFile = fopen ("/Users/godlikephill/Desktop/detection.txt","a");
     //fprintf (pFile, "f \n");
     for(int i=0; i<CenterPtsX.size();i++){
         double x = CenterPtsX[i];
         double y = CenterPtsY[i];
         mea.push_back(Point2d(x,y));     
         fprintf (pFile, "%f %f\n",x,y);
     }
     if(CenterPtsX.size()!=0){
        tracker->Update(mea);
        //tracker->pos = min_pos;
     }

     //fclose (pFile);
}




//--------------------------------------------------------
// return protected variable
//--------------------------------------------------------

VdynePointCloud DataProcessControl::getClusterPointCloud(){
  return ClusterPointCloud;
}


vector<double> DataProcessControl::getDistance(){
  return distance;
}


vector<double> DataProcessControl::getCenterPtsX(){
  return CenterPtsX;
}

vector<double> DataProcessControl::getCenterPtsY(){
  return CenterPtsY;
}


vector<double> DataProcessControl::getDeltaX(){
  return DeltaX;
}

vector<double> DataProcessControl::getDeltaY(){
  return DeltaY;
}

double DataProcessControl::getMinDistance(){
  return dist_min;
}

VdynePointCloud DataProcessControl::getMinCluster(){
  int min_pos = std::distance(std::begin(distance),min_element(std::begin(distance),std::end(distance)));
  //qDebug("size is %d, %d",distance.size(),clusterpc.size());
  //qDebug("min is %d", min_pos);
  if(clusterpc.size() !=0)
  return  clusterpc[min_pos];
}

int  DataProcessControl::getMinPos(){
    if(clusterpc.size() !=0)
    return  min_pos;
}

double  DataProcessControl::getMinX(){
    if(clusterpc.size() !=0)
    return  CenterPtsX[min_pos];
}

double  DataProcessControl::getMinY(){
    if(clusterpc.size() !=0)
    return  CenterPtsY[min_pos];
}




void DataProcessControl::getTrackingRMSE(){

   pFile = fopen ("/Users/godlikephill/Desktop/rmse.txt","a");
    double rmsey = 0;
    double rmsex = 0;
    int n = tracker->tracks.size();
    for(size_t i=0;i< n;i++)
    {
       rmsex = rmsex+(tracker->tracks[i]->rmse_x)*(tracker->tracks[i]->rmse_x);
       rmsey = rmsey+(tracker->tracks[i]->rmse_y)*(tracker->tracks[i]->rmse_y);
        //rmsex = sqrt((tracker->tracks[i]->rmse_x)*(tracker->tracks[i]->rmse_x));
        //rmsey = sqrt((tracker->tracks[i]->rmse_y)*(tracker->tracks[i]->rmse_y));
        //fprintf (pFile, "%f %f\n",rmsex,rmsey);
     }

rmsex = sqrt(rmsex/n);
rmsey = sqrt(rmsey/n);
fprintf (pFile, "%f %f\n",rmsex,rmsey);
fclose (pFile);
//qDebug("%f and %f",rmsex,rmsey);
}
