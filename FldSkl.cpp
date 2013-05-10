
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include <cmath>
using namespace std;

const float VOXEL_SIZE = 0.028f;						// 空间分块的边长， 探索步长时以此为单位
const float STEPLENGTH_RATIO = 0.05f;				// 在探索步长时，大步长比小步长所得邻域点数增长的比例阈值(小于它则不再扩大步长)
const float GEN_BRANCH_LENGTH_RATIO = 1.0 / 15.0f;		    // 某个象限的向量长度占所有象限向量长度和的比例，大于此时, 视为一个新的分支
const float GEN_BRANCH_LENGTH_SECOND = 1.0f / 16.0f;
const float GEN_BRANCH_LENGTH_THIRD = 1.0f / 17.0f;
const int MAX_STEP_LENGTH = 10;
const int NUM_DIRECTIONS = 26;

struct ThreeNums
{
	ThreeNums()
	{
		_x = _y = _z = 0;
	}

	ThreeNums(int x, int y, int z)
	{
		_x = x;
		_y = y;
		_z = z;
	}
	int _x, _y , _z;
};

ThreeNums operator+ (const ThreeNums& lhs, const ThreeNums& rhs)
{
	return ThreeNums(lhs._x + rhs._x, lhs._y + rhs._y, lhs._z + rhs._z);
}

// represent a direction between parent point to child point
struct Float3
{
	Float3()
	{
		_x = _y = _z = 0.0f;
	}

	Float3(float x, float y, float z)
	{
		_x = x;
		_y = y;
		_z = z;
	}

	Float3& operator +=(const Float3& rhs)
	{
		this->_x += rhs._x;
		this->_y += rhs._y;
		this->_z += rhs._z;
		return *this;
	}

	float length() const
	{
		return sqrt(_x * _x + _y * _y + _z * _z);
	}

	Float3 identity() const
	{
		float len = length();
		if(len == 0)
			return Float3(0.0f, 0.0f, 0.0f);
		return Float3(_x / len, _y / len, _z / len);
	}

	float _x, _y, _z;
};

float operator* (const Float3& lhs, const Float3& rhs)
{
	return lhs._x * rhs._x + lhs._y * rhs._y + lhs._z* rhs._z;
}

Float3 operator* (const Float3& lhs, float rhs)
{
	return Float3(lhs._x * rhs, lhs._y * rhs, lhs._z * rhs);
}

Float3 operator* (float lhs, const Float3& rhs)
{
	return Float3(lhs * rhs._x , lhs * rhs._y, lhs * rhs._z);
}

Float3 operator- (const Float3& lhs)
{
	return Float3(-lhs._x, -lhs._y, -lhs._z);
}

Float3 operator+ (const Float3& lhs, const Float3& rhs)
{
	return Float3(lhs._x + rhs._x, lhs._y + rhs._y, lhs._z + rhs._z);
}

Float3 operator- (const Float3& lhs, const Float3& rhs)
{
	return Float3(lhs._x - rhs._x, lhs._y - rhs._y, lhs._z - rhs._z);
}

// represent a small set of points
class Voxel
{
public:
	Voxel()
	{
		_isDisabled = false;
		_isMarked = false;
	}

	inline vector<Float3>& points()
	{
		return _points;
	}

	inline bool isDisabled() const
	{
		return _isDisabled;
	}

	inline void disable()
	{
		_isDisabled = true;
	}

	inline void setMark(bool mark)
	{
		_isMarked = mark;
	}

	inline bool isMarked() const
	{
		return _isMarked;
	}

private:
	vector<Float3> _points;
	bool _isDisabled;
	bool _isMarked;
};

class SkeletonGenerator
{
public:	

	// tree node
	struct TreeNode
	{
		TreeNode(const Float3& pt, float radius, Float3 fd) : 
			_pt(pt), _radius(radius), _fd(fd)
		{
		}
		vector<TreeNode*> _vecChildren;		
		Float3 _pt;
		float _radius;
		Float3 _fd;
	};
	// ------------ public methods -------------

	SkeletonGenerator()
	{
		_minX = _minY = _minZ = 1e38;
		_maxX = _maxY = _maxZ = -1e38;
	}
	~SkeletonGenerator()
	{
		for(int i = 0; i < _length; i++)
		{
			for(int j = 0; j < _height; j++)
			{
				delete [] _space[i][j];
			}
			delete[] _space[i];
		}
		delete[] _space;

		recursiveDelete(pRootNode);
	}

	void readToPointPart(fstream &ifs)
	{
		string str;
		while(str != "end_header")
		{
			getline(ifs, str, '\n');
		}
	}

	void readPointCloud(const string& filename)
	{
		cout << "Reading point cloud to program..." << endl;
		fstream ifs(filename.c_str());
		if(!ifs)
			throw;
		readToPointPart(ifs);
		findBoundry(ifs);
		ifs.close();
		ifs.open(filename.c_str());
		readToPointPart(ifs);
		readToVoxels(ifs);
		ifs.close();
	}
	void generate()
	{
		cout << "Retrieving skeleton from the data..." << endl;

//		int stepLength = getStepLength(rootPoint);
		pRootNode = new TreeNode(rootPoint, 0, Float3(0.0f, 1.0f, 0.0f));
		recursiveGenerate(pRootNode);		
	}

	void outputPly(const string& filename)
	{
		cout << "Outputting the skeleton data to file..." << endl;
		fstream ofs(filename.c_str(), ios::out);
		if(!ofs)
			throw;
		
		ofs << "ply" << endl;
		ofs << "format ascii 1.0" << endl;
		ofs << "comment VCGLIB generated" << endl;
		int pointCount = getPointCount(pRootNode);
		ofs << "element vertex " << pointCount << endl;
		ofs << "property float x" << endl << "property float y" << endl << "property float z" << endl;		
		ofs << "element edge " << pointCount - 1 << endl;
		ofs << "property   int32   vertex1" << endl 
			<< "property   int32   vertex2" << endl 
			<< "property   uint8   red" << endl 
			<< "property   uint8   green" << endl 
			<< "property   uint8   blue" << endl;
		ofs << "end_header" << endl;
		
		
		vector<pair<int, int> > vecPairs;
		recursiveOutputPly(pRootNode, ofs, 0, vecPairs);	
		outputEdges(ofs, vecPairs);

		cout << "Process done." << endl;
		ofs.close();
	}

	void outputSkl(const string& filename)
	{
		cout << "Outputting the skeleton data to file..." << endl;
		fstream ofs(filename.c_str(), ios::out);
		if(!ofs)
			throw;
		recursiveOutputSkl(pRootNode, ofs, 0);	
		cout << "Process done." << endl;
		ofs.close();
	}

private:
	Voxel**** _space;
	float _minX, _minY, _minZ;
	float _maxX, _maxY, _maxZ;
	int _length, _width, _height;
	Float3 rootPoint;
	TreeNode* pRootNode;

	//-------------- auxiliary functions -----------------------

	void findBoundry(fstream& ifs)
	{
		char buffer[100];
		// read the point cloud data, and find the boundaries of the data.
		while(ifs.getline(buffer, 100))
		{
			istringstream line(buffer);
			Float3 pt;
			line >> pt._x >> pt._y >> pt._z;			
			if(pt._x < _minX)
				_minX = pt._x;
			if(pt._y < _minY)
				_minY = pt._y;
			if(pt._z < _minZ)
				_minZ = pt._z;
			if(pt._x > _maxX)
				_maxX = pt._x;
			if(pt._y > _maxY)
				_maxY = pt._y;
			if(pt._z > _maxZ)
				_maxZ = pt._z;
		}

		// then divide the space into many voxels
		_length = (_maxX + VOXEL_SIZE - _minX) / VOXEL_SIZE + 1;
		_width = (_maxZ + VOXEL_SIZE - _minZ) / VOXEL_SIZE + 1;
		_height = (_maxY + VOXEL_SIZE - _minY) / VOXEL_SIZE + 1;
		_space = new Voxel***[_length];
		for(int i = 0; i < _length; i++)
		{
			_space[i] = new Voxel**[_height];
			for(int j = 0; j < _height; j++)
			{
				_space[i][j] = new Voxel*[_width];
				for(int k = 0; k < _width; k++)
				{
					_space[i][j][k] = new Voxel();
				}
			}
		}
	}

	void readToVoxels(fstream& ifs)
	{
		char buffer[100];
		while(ifs.getline(buffer, 100))
		{
			istringstream line(buffer);
			Float3 pt;			
			line >> pt._x >> pt._y >> pt._z;			
			if(pt._y == _minY)
				rootPoint = pt;
			// then put the point to corresponding voxel
			getVoxel(pt)->points().push_back(pt);
		}
	}

	inline Voxel* getVoxel(Float3 pt)
	{
		pt._x = max(min(pt._x, _maxX), _minX);
		pt._y = max(min(pt._y, _maxY), _minY);
		pt._z = max(min(pt._z, _maxZ), _minZ);
		int px, py, pz;
		px = (int)((pt._x - _minX) / VOXEL_SIZE);
		py = (int)((pt._y - _minY) / VOXEL_SIZE);
		pz = (int)((pt._z - _minZ) / VOXEL_SIZE);		
		return _space[px][py][pz];
	}

	ThreeNums getVoxelIndex(const Float3& pt)
	{
		ThreeNums tn;
		tn._x = (int)((pt._x - _minX) / VOXEL_SIZE);
		tn._y = (int)((pt._y - _minY) / VOXEL_SIZE);
		tn._z = (int)((pt._z - _minZ) / VOXEL_SIZE);
		return tn;
	}

	inline vector<Float3> getNearbyPointsByFlooding(const Float3& pt)
	{
		ThreeNums sixDirs[6] = {ThreeNums(-1, 0, 0), ThreeNums(1, 0, 0), 
			ThreeNums(0, -1, 0), ThreeNums(0, 1, 0),
			ThreeNums(0, 0, -1), ThreeNums(0, 0, 1)};
		ThreeNums tn = getVoxelIndex(pt);
		queue<ThreeNums> prev;
		queue<ThreeNums> cur;		
		queue<ThreeNums> markRecords;
		int prevTotalSum = 0;
		int curTotal = 0;
		int curStepLength = 0;
		
		_space[tn._x][tn._y][tn._z]->setMark(true);
		markRecords.push(tn);
		prev.push(tn);

		vector<Float3> ret;
		while(!prev.empty() || !cur.empty())
		{			
			ThreeNums tmp = prev.front();
			prev.pop();
			// first add points to ret.
			Voxel* pVoxel = _space[tmp._x][tmp._y][tmp._z];
			curTotal += pVoxel->points().size();
			for(int i = 0; i < pVoxel->points().size(); i++)
			{
				ret.push_back(pVoxel->points()[i]);
			}
			// then find neighbor voxels to append into the queue.
			for(int i = 0; i < 6; i++)
			{
				ThreeNums next = tmp + sixDirs[i];
				next._x = max(min(_length - 1, next._x), 0);
				next._y = max(min(_height - 1, next._y), 0);
				next._z = max(min(_width - 1, next._z), 0);
				Voxel* pV = _space[next._x][next._y][next._z];
				if(pV->isMarked() || pV->isDisabled() || pV->points().empty())
					continue;
				_space[next._x][next._y][next._z]->setMark(true);
				markRecords.push(next);
				cur.push(next);
			}			
			if(prev.empty())
			{
				if(++curStepLength > MAX_STEP_LENGTH)
					break;
				if(prevTotalSum != 0)
				{
					if((float)curTotal / (float)prevTotalSum < STEPLENGTH_RATIO)
						break;					
				}
				prevTotalSum += curTotal;
				curTotal = 0;			
				while(!cur.empty())
				{										
					prev.push(cur.front());
					cur.pop();
				}
			}
		}

		// unmark the voxels
		while(!markRecords.empty())
		{
			ThreeNums tmp = markRecords.front();
			_space[tmp._x][tmp._y][tmp._z]->setMark(false);
			markRecords.pop();
		}
		return ret;
	}

	// used to calculate Manhattan distance
	void getSequence(vector<ThreeNums>& vec, int n)
	{
		for(int i = 1; i <= n; i++)
		{
			getSubSequence(vec, i);
		}
	}

	void getSubSequence(vector<ThreeNums>& vec, int n)
	{
		for(int i = 0; i <= n; i++)
		{
			for(int j = 0; j <= n - i; j++)
			{
				vec.push_back(ThreeNums(i, j, n - i - j));
			}
		}
	}

	void generateSymmetric(const vector<ThreeNums>& vec, vector<ThreeNums>& result)
	{
		for(int i = 0; i < vec.size(); i++)
		{
			result.push_back(vec[i]);
			if(vec[i]._y != 0)
			{
				result.push_back(ThreeNums(vec[i]._x, -vec[i]._y, vec[i]._z));
				if(vec[i]._z != 0)
					result.push_back(ThreeNums(vec[i]._x, -vec[i]._y, -vec[i]._z));
			}
			if(vec[i]._z != 0)
				result.push_back(ThreeNums(vec[i]._x, vec[i]._y, -vec[i]._z));
			if(vec[i]._x != 0)
			{
				result.push_back(ThreeNums(-vec[i]._x, vec[i]._y, vec[i]._z));
				if(vec[i]._y != 0)
				{
					result.push_back(ThreeNums(-vec[i]._x, -vec[i]._y, vec[i]._z));
					if(vec[i]._z != 0)
						result.push_back(ThreeNums(-vec[i]._x, -vec[i]._y, -vec[i]._z));
				}
				if(vec[i]._z != 0)
					result.push_back(ThreeNums(-vec[i]._x, vec[i]._y, -vec[i]._z));
			}
		}
	}

	// calculate step length by
	//int getStepLength(const Float3& seed)
	//{
	//	// 1. first take the boundaries into consideration
	//	ThreeNums tn = getVoxelIndex(seed);		
	//	// 2. second try to expand to find the most proper step length
	//	int prevNum = getNearbyPoints(seed, 0).size();

	//	int stepLength;
	//	for(stepLength = 1; stepLength < MAX_STEP_LENGTH; stepLength++)
	//	{
	//		int curNum = getNearbyPoints(seed, stepLength).size();
	//		float ratio = (float)(curNum - prevNum) / (float)prevNum;
	//		if(ratio < STEPLENGTH_RATIO)
	//		{
	//			stepLength--;
	//			break;
	//		}
	//		prevNum = curNum;
	//	}
	//	return stepLength;
	//}

	vector<Float3> getDirections(const Float3& seed, const vector<Float3>& points, Float3 fd)
	{
		// first calculate the direction of each seed-to-point, then put them into eight quadrants
		vector<vector<Float3> > dirSets;
		vector<Float3> ret;
		for(int i = 0; i < NUM_DIRECTIONS; i++)
		{
			dirSets.push_back(vector<Float3>());
		}
		for(int i = 0; i < points.size(); i++)
		{
			Float3 dir(points[i]._x - seed._x, points[i]._y - seed._y, points[i]._z - seed._z);
			if(dir * fd <= 0)
				continue;
			int index = calDirectionSet(dir);
			if(index == -1)
				continue;
			dirSets[index].push_back(dir);
		}
		vector<Float3> dirSums;
		for(int i = 0; i < dirSets.size(); i++)
		{
			Float3 sum;
			for(int j = 0; j < dirSets[i].size(); j++)
			{
				sum += dirSets[i][j];
			}
			dirSums.push_back(sum);
		}
		float totalLength = 0.0f;		
		for(int i = 0; i < dirSums.size(); i++)
		{			
			totalLength += dirSums[i].length();
		}
		for(int i = 0; i < dirSums.size(); i++)
		{			
			if(dirSums[i].length() / totalLength >= GEN_BRANCH_LENGTH_RATIO)
				ret.push_back(1.0f / (float)dirSets[i].size() * dirSums[i]);
		}
		if(ret.size() == 0)
		{
			for(int i = 0; i < dirSums.size(); i++)
			{				
				if(dirSums[i].length() / totalLength >= GEN_BRANCH_LENGTH_SECOND)
					ret.push_back(1.0f / (float)dirSets[i].size() * dirSums[i]);
			}
		}
		if(ret.size() == 0)
		{
			for(int i = 0; i < dirSums.size(); i++)
			{				
				if(dirSums[i].length() / totalLength >= GEN_BRANCH_LENGTH_THIRD)
					ret.push_back(1.0f / (float)dirSets[i].size() * dirSums[i]);
			}
		}
		return ret;
	}

	// determine which set the input direction belongs to.(according to the direction array)
	int calDirectionSet(const Float3& dir)
	{
		Float3 d = dir.identity();		
		static Float3 dirs[26] = {
			Float3(0.0f, 1.0f, 0.0f).identity(), 
			Float3(-1.0f, 1.0f, -1.0f).identity(), 
			Float3(0.0f, 1.0f, -1.0f).identity(), 
			Float3(1.0f, 1.0f, -1.0f).identity(), 
			Float3(-1.0f, 1.0f, 0.0f).identity(), 			
			Float3(1.0f, 1.0f, 0.0f).identity(), 
			Float3(-1.0f, 1.0f, 1.0f).identity(), 
			Float3(0.0f, 1.0f, 1.0f).identity(), 
			Float3(1.0f, 1.0f, 1.0f).identity(), 
			Float3(-1.0f, 0.0f, -1.0f).identity(), 
			Float3(0.0f, 0.0f, -1.0f).identity(), 
			Float3(1.0f, 0.0f, -1.0f).identity(), 
			Float3(-1.0f, 0.0f, 0.0f).identity(), 
			Float3(1.0f, 0.0f, 0.0f).identity(), 
			Float3(-1.0f, 0.0f, 1.0f).identity(), 
			Float3(0.0f, 0.0f, 1.0f).identity(), 
			Float3(1.0f, 0.0f, 1.0f).identity(), 
			Float3(-1.0f, -1.0f, -1.0f).identity(), 
			Float3(0.0f, -1.0f, -1.0f).identity(), 
			Float3(1.0f, -1.0f, -1.0f).identity(), 
			Float3(-1.0f, -1.0f, 0.0f).identity(), 
			Float3(0.0f, -1.0f, 0.0f).identity(), 
			Float3(1.0f, -1.0f, 0.0f).identity(), 
			Float3(-1.0f, -1.0f, 1.0f).identity(), 
			Float3(0.0f, -1.0f, 1.0f).identity(), 
			Float3(1.0f, -1.0f, 1.0f).identity()
			};

		float maxCos = -1.0f;
		int maxIndex = -1;
		for(int i = 0; i < NUM_DIRECTIONS; i++)
		{
			if(d * dirs[i] > maxCos)
			{
				maxCos = d * dirs[i];
				maxIndex = i;
			}
		}

		return maxIndex;
	}

	void recursiveGenerate(TreeNode* subroot)
	{		
		vector<Float3> points = getNearbyPointsByFlooding(subroot->_pt);
		if(points.empty())
			return;
		vector<Float3> dirs = getDirections(subroot->_pt, points, subroot->_fd);
		for(int i = 0; i < dirs.size(); i++)
		{
			Float3 pt;
			pt._x = subroot->_pt._x + dirs[i]._x;
			pt._y = subroot->_pt._y + dirs[i]._y;
			pt._z = subroot->_pt._z + dirs[i]._z;
			pt._x = max(min(pt._x, _maxX), _minX);
			pt._y = max(min(pt._y, _maxY), _minY);
			pt._z = max(min(pt._z, _maxZ), _minZ);
			if(getVoxel(pt)->isDisabled())
				continue;						

			// to discover the radius of branch
			Float3 mid = (subroot->_pt + pt) * 0.5f;
			Float3 radPt1, radPt2;
			radPt1._z = pt * dirs[i] / dirs[i]._z;
			radPt2 = 2.0f * mid -radPt1;
			// 2 direction vectors to find out the radius of branch.
			Float3 radDir1 = (radPt1 - mid).identity(), radDir2 = (radPt2 - mid).identity();
			int radius1 = 0, radius2 = 0;
			// first iterate the first direction
			while(true)
			{
				Float3 tmp = mid + radDir1 * VOXEL_SIZE * (radius1 + 1);
				if(getVoxel(tmp)->isDisabled() || getVoxel(tmp)->points().empty())
					break;
				radius1 ++;
			}
			// then iterate the second direction
			while(true)
			{
				Float3 tmp = mid + radDir2 * VOXEL_SIZE * (radius2 + 1);
				if(getVoxel(tmp)->isDisabled() || getVoxel(tmp)->points().empty())
					break;
				radius2 ++;
			}
			int totalRadius;
			if(getVoxel(mid)->isDisabled() || getVoxel(mid)->points().empty())
				totalRadius = radius1 + radius2;
			else
				totalRadius = radius1 + radius2 + 1;

			subroot->_vecChildren.push_back(new TreeNode(pt, totalRadius * VOXEL_SIZE / 2.0f, dirs[i].identity()));

			// Disable the voxels used to avoid the cycle.

			disableVoxelsOnPath(totalRadius * VOXEL_SIZE / 2.0f, subroot->_pt, pt);
		}
		for(int i = 0; i < subroot->_vecChildren.size(); i++)
			recursiveGenerate(subroot->_vecChildren[i]);
	}

	void disableVoxelsOnPath(float radius, const Float3& start, const Float3& end)
	{
		Float3 dir = (end - start).identity();
		float minX = min(start._x, end._x), minY = min(start._y, end._y), minZ = min(start._z, end._z);
		float maxX = max(start._x, end._x), maxY = max(start._y, end._y), maxZ = max(start._z, end._z);
		minX -= radius; minY -= radius; minZ -= radius;
		maxX += radius; maxY += radius; maxZ += radius;
		ThreeNums tn1 = getVoxelIndex(Float3(minX, minY, minZ));
		ThreeNums tn2 = getVoxelIndex(Float3(maxX, maxY, maxZ));
		for(int i = tn1._x; i <= tn2._x; i++)
		{
			for(int j = tn1._y; j <= tn2._y; j++)
			{
				for(int k = tn1._z; k <= tn2._z; k++)
				{
					Float3 tmp = Float3((i + 0.5f) * VOXEL_SIZE + _minX, (j + 0.5f) * VOXEL_SIZE + _minY, (k + 0.5f) * VOXEL_SIZE + _minZ);
					if((tmp - start) * dir < 0 || (end - tmp) * dir < 0)
						continue;
					float t = (tmp * dir - start * dir) / (dir * dir);
					Float3 intersec = start + dir * t;
					Float3 rad = tmp - intersec;
					if(rad.length() <= radius)
						getVoxel(tmp)->disable();
				}
			}
		}
	}

	void recursiveOutputPly(TreeNode* subroot, fstream& ofs, int lineNum, vector<pair<int, int> >& vecPairs)
	{
		static int curLineNum = 0;
		if(subroot == 0)
			return;
		
		ofs << subroot->_pt._x << " " << subroot->_pt._y << " " << subroot->_pt._z << endl;		

		if(subroot != pRootNode)
			vecPairs.push_back(pair<int, int>(lineNum - 1, curLineNum));
		curLineNum ++;
		
		if(subroot->_vecChildren.size() == 0)
			return;
		int ln = curLineNum;
		for(int i = 0; i < subroot->_vecChildren.size(); i++)
		{
			recursiveOutputPly(subroot->_vecChildren[i], ofs, ln, vecPairs);
		}
	}

	void recursiveOutputSkl(TreeNode* subroot, fstream& ofs, int lineNum)
	{
		static int curLineNum = 0;
		if(subroot == 0)
			return;
				
		ofs << lineNum << " " << subroot->_pt._x << " " << subroot->_pt._y << " " << subroot->_pt._z << 
			" " << 0.1f << endl;
		
		curLineNum ++;
		
		if(subroot->_vecChildren.size() == 0)
			return;
		int ln = curLineNum;
		for(int i = 0; i < subroot->_vecChildren.size(); i++)
		{
			recursiveOutputSkl(subroot->_vecChildren[i], ofs, ln);
		}
	}

	void outputEdges(fstream& ofs, vector<pair<int, int> > vecPairs)
	{
		for(int i = 0; i < vecPairs.size(); i++)
		{
			ofs << vecPairs[i].first << " " << vecPairs[i].second << " " << 255 << " " << 255 << " " << 255 << endl;
		}
	}

	void recursiveDelete(TreeNode* subroot)
	{
		if(subroot == 0)
			return;
		for(int i = 0; i < subroot->_vecChildren.size(); i++)
			recursiveDelete(subroot->_vecChildren[i]);
		delete subroot;
	}	

	int getPointCount(TreeNode* subroot) const
	{
		if(subroot == 0)
			return 0;
		int count = 1;
		for(int i = 0; i < subroot->_vecChildren.size(); i++)
			count += getPointCount(subroot->_vecChildren[i]);
		return count;	
	}
};

int main()
{
	SkeletonGenerator sg;
	sg.readPointCloud("Tree.ply");
	sg.generate();
	sg.outputPly("skeleton.ply");
	sg.outputSkl("skeleton.txt");
	return 0;
}
