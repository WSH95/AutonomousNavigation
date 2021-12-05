#ifndef GRID_H
#define GRID_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <deque>
using namespace std;

class GridMapTest;

class GridMapTest{
public:
    //初始化删格地图  输入：图像宽高 删格宽高   初始化各个删格状态为2（未知）
    //构造函数不能返回特定类型的参数
    GridMapTest(int WidthGridNum = 120,int HeightGridNum = 120){
      if (((WidthGridNum/2)*2)!=WidthGridNum || ((HeightGridNum/2)*2)!=HeightGridNum)
        cerr<<"输入删格必须为偶数！重新定义删格！"<<endl;
      deque<int> XCoordState (WidthGridNum, 2);
      deque<deque<int> > YCoord(HeightGridNum, XCoordState);
      _YCoord = YCoord;
      _laser_x = WidthGridNum/2-1;
      _laser_y = HeightGridNum/2-1;
    }


    //以读文件的方式创建对象
    GridMapTest(string adress){
      deque<int> test_line;
      deque<deque<int> > test_grid;
      ifstream infile(adress);
      if (!infile)  { cerr<<"读取文件失败!"<<endl; }
      else{
        int reader,Ynum = 0;
        //读取激光雷达数据
        infile >> _laser_x >> _laser_y;

        //读取文件到二维deque
        while(infile >> reader){
          //行内的操作
          //cout << reader << endl;
          if(reader == 0 || reader == 1 || reader == 2){
            test_line.push_back(reader);
          }
            //换行时的操作
          else if(reader == 9) {
            if (Ynum>0 &&test_line.size() != test_grid[Ynum-1].size()){
              cerr<<"部分文件读取错误!"<<endl;
            }//防止某行漏读
            test_grid.push_back(test_line);
            test_line.clear();
            Ynum++;
          }
          else cerr << "文件读取错误！" << endl;
        }//整个文件读取完成

        //初始化private成员
        _YCoord = test_grid;
        cout<<"文件读取完成!"<<" 行数： "<<this->HeightNum()<<"列数： "<<this->WidthNum()
            << endl<< "激光雷达原点： x："<< _laser_x  << "   y:" << _laser_y <<endl;
      }
    }

    //构析函数
    /*
    ~GridMapTest(){delete &_YCoord;delete &_XCoordState;
      delete &_WidthGridNum; delete &_HeightGridNum;
    }
     */

    //读取某个特定删格的状态 从第一个删格开始
    int GetGridState(int x,int y){ return _YCoord[y-1][x-1];}

    //以激光原点为参照系读取删格状态
    int GetGridCorLaser(int x,int y){
      //cout << "_YCoord[_laser_y - y -1][_laser_x + x -1];  " << _laser_y - y -1
      //     <<"  "<< _laser_x + x -1 << endl;
      return _YCoord[_laser_y - y -1][_laser_x + x -1];
    }

    //以向量形式返回删格
    vector<vector<int> > GetGridVectorTest(){
      vector<int> a;
      vector<vector<int> > YCoord_Vector(this->HeightNum(),a);
      for(int j=0;j<this->HeightNum();j++){
        for(int i=0;i<this->WidthNum();i++){
          YCoord_Vector[j].push_back(this->_YCoord[j][i]);
        }
      }
      return YCoord_Vector;
    }

    //读取删格的长和宽
    int WidthNum(){ return _YCoord[0].size(); }
    int HeightNum(){ return _YCoord.size(); }

    //删格规模扩容  在原有删格坐标系下的 极限坐标(可以为负数)
    bool EnlargeGrid(int x,int y){
      //判断X是否超出范围，超出范围就扩容
      if(x>(int)_YCoord[0].size()){
        int deltaX = x - _YCoord[0].size();
        //x方向补齐
        while(deltaX){
          //每行都增加一个删格
          int col_flag = _YCoord.size();
          while(col_flag){_YCoord[col_flag-1].push_back(2);  col_flag--;}
          deltaX--;
        }
        //cout << "x正方向扩容" << endl;
      }
      else if(x<=0){
        int deltaX = (-x)+1;
        while(deltaX){
          //cout << "x负方向扩容" << deltaX <<endl;
          int col_flag = _YCoord.size();
          while(col_flag){_YCoord[col_flag-1].push_front(2);  col_flag--; }
          deltaX--;
        }
        _laser_x = _laser_x + (-x+1);
        //cout << "x负方向扩容" << endl;
      }
      //判断Y是否超出范围，超出范围就扩容
      if(y>(int)_YCoord.size()){
        deque<int> LineTemp(_YCoord[0].size(),2);
        int deltaY = y - _YCoord.size();
        //Y方向补齐
        while(deltaY){_YCoord.push_back(LineTemp);  deltaY--;}
        //cout << "y正方向扩容" << endl;
      }
      else if(y<=0){
        deque<int> LineTemp(_YCoord[0].size(),2);
        int deltaY = (-y)+1;
        while(deltaY){_YCoord.push_front(LineTemp);  deltaY--;}
        _laser_y = _laser_y + (-y+1);
        //cout << "y负方向扩容" << endl;
      }
      return true;
    }


    //修改某个特定删格的状态 从第一个删格开始  x坐标 y坐标
    bool ModifyGridState(int x,int y,int state){
      //cout << "修改的删格：  x：" << x << "  y："  << y << endl;
      if (state != 0 && state != 1 && state != 2){
        cerr << "input state is illegal!" << endl;
        return false;
      }
      else{
        _YCoord[y-1][x-1] = state;
        return true;
      }
    }


    //以雷达为原点的参考系修改特定删格状态
    bool ModifyGridCorLaser(int x,int y,int state){
      //cout <<"ModifyGridCorLaser("<<x << ","<<y<< endl;
      //cout <<"EnlargeGrid("<<_laser_x + x << ","<<_laser_y - y<< endl;
      EnlargeGrid(_laser_x + x, _laser_y - y);
      //cout <<" ModifyGridState("<< _laser_x + x << "," <<_laser_y - y << endl;
      ModifyGridState(_laser_x+x,_laser_y-y,state); //此时的_laser_x和_laser_y已经与上一行不同
    }

    //某删格在删格原点下的坐标(从1开始)转化为在激光雷达下的坐标
    bool GridCoord2LaserCoord(int GridCordX,int GridCordY,int &LaserCordX,int &LaserCordY){
      LaserCordX = GridCordX - _laser_x;
      LaserCordY = -GridCordY + _laser_y;
    }


    bool SaveGrid(string filename = "grid.txt"){
      //创建文件对象
      ofstream outfile("../saved/"+filename);

      if(!outfile){cerr << " 文件打开失败 "<<endl;  return false;}
      //写删格中的激光位置
      outfile << _laser_x << " " << _laser_y << endl;

      for(int j=0;j<_YCoord.size();j++){
        //对每一行y
        for(int i=0;i<_YCoord[0].size();i++){
          //对每行的每个删格x
          outfile << GetGridState(i+1,j+1) << " ";//以空格为每个删格的间隔
        }
        outfile << 9 << endl;//以9为每行的间隔
      }
      //写入文件
      return true;
    }

private:
    int _laser_x;
    int _laser_y;

    deque<deque<int> >  _YCoord;    //标号是Y序列 ,纵坐标，行数

};

#endif
