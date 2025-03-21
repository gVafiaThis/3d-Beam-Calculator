#pragma once
#include <set>
#include <vector>
#include <map>
#include <array>
#include <math.h>
#include <numeric>
#include <memory>
#include <Eigen/SparseLU>
#include <Eigen/Dense>
#include<Eigen/SparseCholesky>	
//#include<Eigen/SparseLU>
//#include<Eigen/SparseQR>
#include<Eigen/SparseQR>
//Make more efficient triplet changes. For now it is a vector that gets reevaluated at analyse.
//Make all stifness matrices only triangular stuff
#include "raylib.h"
#include <algorithm>
//float error tolerance
#define ERR_TOLERANCE 0.000000001
#define RENDER_SCALING_FACTOR 0.1

//#define DEBUG_PRINTS



namespace Beams {

	struct Node {
		float x, y, z;
		float xRender;
		float yRender;
		float zRender;

		size_t matrixPos; //for matrix use
		
		int pos; //position of Coords in the points vector;

		bool free_flag = true; //DOFs not used in stifness matrix
		std::set<Eigen::Index> inElements;
		Node(float x_, float y_, float z_, size_t id_) {
			x = x_;
			y = y_;
			z = z_;

			xRender = x_ * RENDER_SCALING_FACTOR;
			yRender = y_ * RENDER_SCALING_FACTOR;
			zRender = z_ * RENDER_SCALING_FACTOR;

			matrixPos = id_;
		}
		/*bool operator< (const Node  & v) {
			if (std::abs(x - v.x) >ERR_TOLERANCE) return x < v.x;
			if (std::abs(y - v.y) >ERR_TOLERANCE) return y < v.y;
			return z < v.z;
		}*/
	};

	bool operator< (Node const& l, Node const& v) {
		if (std::abs(l.x - v.x) > ERR_TOLERANCE) return l.x < v.x;
		if (std::abs(l.y - v.y) > ERR_TOLERANCE) return l.y < v.y;
		return l.z < v.z;
	}

	//UNUSED -> maybe for map with common local stiffness vectors mapped to element length. Not gonna b faster
	class Length {
		float l;

		bool operator< (const Length& v) {
			if (std::abs(l - v.l) > ERR_TOLERANCE) return l < v.l;
			return false;
		}
	};


	class NodeContainer {
		std::vector<Node> Nodes;
		std::set<size_t> deleted;


		Node& get_NotDeleted(size_t pos) {
			auto it = deleted.find(pos);
			auto endIt = deleted.end();
			while (it != endIt) {
				pos++;
				it = deleted.find(pos);
			}
			return Nodes[pos];
		}

	public:
		class notDeleted_const_iterator {

		public:
			using iterator_category = std::forward_iterator_tag; 
			using difference_type = std::ptrdiff_t;
			using value_type = const Node;
			using pointer =  const Node*;
			using reference =  const Node&;

			explicit notDeleted_const_iterator(const std::vector<Node>& nodes, const std::set<size_t>& deleted, size_t pos) : m_nodes(nodes), m_deleted(deleted), m_pos(pos) {}

			reference operator*() const { return m_nodes[m_pos]; }
			//pointer operator->() { return m_ptr; }
			notDeleted_const_iterator& operator++() { 
				//m_ptr++; 
				m_pos++;
				while (m_deleted.find(m_pos) != m_deleted.end()) {
					m_pos++;
					//m_ptr++;
				}
				
				return *this; 
			}
			notDeleted_const_iterator operator++(int) { notDeleted_const_iterator tmp = *this; ++(*this); return tmp; }
			friend bool operator== (const notDeleted_const_iterator& a, const notDeleted_const_iterator& b) { return a.m_pos == b.m_pos; };
			friend bool operator!= (const notDeleted_const_iterator& a, const notDeleted_const_iterator& b) { return a.m_pos != b.m_pos; };
		private:	
			//pointer m_ptr;
			const std::vector<Node> m_nodes;
			const std::set<size_t>& m_deleted;
			size_t m_pos;
		};
		
		notDeleted_const_iterator begin() const {  

			return notDeleted_const_iterator(Nodes, deleted, 0+(!size() * Nodes.size())); 
		}
		notDeleted_const_iterator end() const { return notDeleted_const_iterator(Nodes, deleted, Nodes.size()); }

		void emplace(Vector3& point) {
			if (deleted.size() > 0) {
				auto firstIt = deleted.begin();
				size_t pos = *firstIt;
				Nodes[pos].x = point.x;
				Nodes[pos].y = point.y;
				Nodes[pos].z = point.z;
				Nodes[pos].xRender = point.x*RENDER_SCALING_FACTOR;
				Nodes[pos].yRender = point.y*RENDER_SCALING_FACTOR;
				Nodes[pos].zRender = point.z*RENDER_SCALING_FACTOR;

				Nodes[pos].matrixPos = -1;
				Nodes[pos].free_flag= true; //extra safety

				deleted.erase(firstIt);
				return;
			}

			Nodes.emplace_back(point.x, point.y, point.z, -1);
			Nodes.back().pos = Nodes.size() - 1;


			return;

		}

		void remove(size_t pos) {
			if (pos >= Nodes.size()) return;
			deleted.insert(pos);
			return;
		}

		//Gets non deleted size
		size_t  size() const {
			return Nodes.size() - deleted.size();
		}

		//Returns nth not deleted node. For iterations
		const Node& get_notDeleted(size_t pos) const {
			auto endIt = deleted.end();
			
			size_t undeleted = (deleted.begin() == deleted.end()) ? 0 :  *deleted.begin();
			for (size_t i = undeleted; i <= pos; i++) {
				if (deleted.find(i) != deleted.end()) pos++;
			}

			return Nodes[pos];

		}

		//Returns Node despite if its deleted or not. For Elements
		const Node& get_byPos(size_t pos) const {
			return Nodes[pos];
		}

		void setFree_fromAll(size_t pos, bool free) {
			Nodes[pos].free_flag = free;
			return;
		}

		bool getFree_fromAll(size_t pos) {
			return Nodes[pos].free_flag;
		}

		void add_InElement_fromAll(size_t pos, Eigen::Index id) {
			Node& node = Nodes[pos];
			node.inElements.insert(id);
			//free gets set from outside here. 
		}

		void remove_InElement_fromAll(size_t pos, Eigen::Index id) {
			Node& node = Nodes[pos];
			node.inElements.erase(id);
			node.free_flag = !node.inElements.size();
		}

		void setMatrixPos(size_t pos, Eigen::Index matPos) {
			get_NotDeleted(pos).matrixPos = matPos;
		}

		void setMatrixPos_fromAll(size_t pos, Eigen::Index matPos) {
			Nodes[pos].matrixPos = matPos;
		}

	};

	class Section {
	public:
		std::vector<size_t> inElements;
		float Area;
		float Ixx;
		float Izz;
		float Iyy;
		float Modulus;
		float G;

		//preCalculated stuff for this section
		float EIz12;
		float EIy12;
		float EIz6;
		float EIy6;
		float EIz4;
		float EIy4;
		float EIz2;
		float EIy2;

		Section() {
			Area = 10;
			Ixx = 10;
			Izz = 10;
			Iyy = 10;
			Modulus = 10;
			G = 10;
			EIz12 = 12 * Modulus * Izz;
			EIy12 = 12 * Modulus * Iyy;
			EIz6 = 6 * Modulus * Izz;
			EIy6 = 6 * Modulus * Iyy;
			EIz4 = 4 * Modulus * Izz;
			EIy4 = 4 * Modulus * Iyy;
			EIz2 = 2 * Modulus * Izz;
			EIy2 = 2 * Modulus * Iyy;
		}

		Section(float _Area, float _Modulus, float _G, float _Ixx, float _Iyy, float _Izz) {
			Area = _Area;
			Izz = _Izz;
			Ixx = _Ixx;


			Iyy = (_Iyy == NULL) ? _Izz : _Iyy;
			Ixx = (_Ixx == NULL) ? 0.001 : _Ixx;//Hackia. Bad 

			Modulus = _Modulus;
			G = _G;
			EIz12 = 12 * Modulus * Izz;
			EIy12 = 12 * Modulus * Iyy;
			EIz6 = 6 * Modulus * Izz;
			EIy6 = 6 * Modulus * Iyy;
			EIz4 = 4 * Modulus * Izz;
			EIy4 = 4 * Modulus * Iyy;
			EIz2 = 2 * Modulus * Izz;
			EIy2 = 2 * Modulus * Iyy;
		}
	};

	class vBeam {
		Eigen::Index id;

		Eigen::SparseMatrix<float> localBmatrix;

		float Len;
		//make properties for common stifness matrices
		size_t sectionId;

		void calc_BMatrixTEST() {
			float Area = 100;
			float Izz = 100;
			float Iyy = 100;
			float Modulus = 210000;
			float Lsq = Len * Len;
			float sixLen = 6 * Len;
			float EIz12 = 12 * Modulus * Izz / pow(Len, 3);
			float EIy12 = 12 * Modulus * Iyy / pow(Len, 3);
			float EIz6 = 6 * Modulus * Izz / pow(Len, 2);
			float EIy6 = 6 * Modulus * Iyy / pow(Len, 2);
			float EIz4 = 4 * Modulus * Izz / Len;
			float EIy4 = 4 * Modulus * Iyy / Len;
			float EIz2 = 2 * Modulus * Izz / Len;
			float EIy2 = 2 * Modulus * Iyy / Len;
			std::vector<Eigen::Triplet<float>> locStiffness_triplets(47);




			float k1 = Modulus * Area / Len;
			float k2 = Modulus * Area / Len;// FIX THIS THIS IS WRONG <--------------------------------------------------
			locStiffness_triplets.emplace_back(0, 0, k1);
			locStiffness_triplets.emplace_back(0, 6, -k1);
			locStiffness_triplets.emplace_back(1, 1, EIz12);
			locStiffness_triplets.emplace_back(1, 5, EIz6);
			locStiffness_triplets.emplace_back(1, 7, -EIz12);
			locStiffness_triplets.emplace_back(1, 11, EIz6);
			locStiffness_triplets.emplace_back(2, 2, EIy12);
			locStiffness_triplets.emplace_back(2, 4, -EIy6);
			locStiffness_triplets.emplace_back(2, 8, -EIy12);
			locStiffness_triplets.emplace_back(2, 10, -EIy6);
			locStiffness_triplets.emplace_back(3, 3, k2);
			locStiffness_triplets.emplace_back(3, 9, -k2);
			locStiffness_triplets.emplace_back(4, 2, -EIy6);
			locStiffness_triplets.emplace_back(4, 4, EIy4);
			locStiffness_triplets.emplace_back(4, 8, EIy6);
			locStiffness_triplets.emplace_back(4, 10, EIy2);
			locStiffness_triplets.emplace_back(5, 1, EIz6);
			locStiffness_triplets.emplace_back(5, 5, EIz4);
			locStiffness_triplets.emplace_back(5, 7, -EIz6);
			locStiffness_triplets.emplace_back(5, 11, EIz2);
			locStiffness_triplets.emplace_back(6, 0, -k1);
			locStiffness_triplets.emplace_back(6, 6, k1);
			locStiffness_triplets.emplace_back(7, 1, -EIz12);
			locStiffness_triplets.emplace_back(7, 5, -EIz6);
			locStiffness_triplets.emplace_back(7, 7, EIz12);
			locStiffness_triplets.emplace_back(7, 11, -EIz6);
			locStiffness_triplets.emplace_back(8, 2, -EIy12);
			locStiffness_triplets.emplace_back(8, 4, EIy6);
			locStiffness_triplets.emplace_back(8, 8, EIy12);
			locStiffness_triplets.emplace_back(8, 10, EIy6);
			locStiffness_triplets.emplace_back(9, 3, -k2);
			locStiffness_triplets.emplace_back(9, 9, k2);
			locStiffness_triplets.emplace_back(10, 2, -EIy6);
			locStiffness_triplets.emplace_back(10, 4, EIy2);
			locStiffness_triplets.emplace_back(10, 8, EIy6);
			locStiffness_triplets.emplace_back(10, 10, EIy4);
			locStiffness_triplets.emplace_back(11, 1, EIz6);
			locStiffness_triplets.emplace_back(11, 5, EIz2);
			locStiffness_triplets.emplace_back(11, 7, -EIz6);
			locStiffness_triplets.emplace_back(11, 11, EIz4);

			localBmatrix.setFromTriplets(locStiffness_triplets.begin(), locStiffness_triplets.end());

		};

		void calc_Len(const Node& N2, const Node& N1) {
			Len = std::pow((N2.x - N1.x), 2) + std::pow((N2.y - N1.y), 2) + std::pow((N2.z - N1.z), 2);
			Len = std::sqrt(Len);
		};

		void calc_BMatrix(const Section section) {
			localBmatrix.data().clear();
			float Area = section.Area;
			float Izz = section.Izz;
			float Iyy = section.Iyy;
			float Modulus = section.Modulus;

			float Lsq = 1 / std::pow(Len, 2);
			float Lcb = 1 / std::pow(Len, 3);
			float invLen = 1 / Len;
			float EIz12 = section.EIz12 * Lcb;
			float EIy12 = section.EIy12 * Lcb;
			float EIz6 = section.EIz6 * Lsq;
			float EIy6 = section.EIy6 * Lsq;
			float EIz4 = section.EIz4 * invLen;
			float EIy4 = section.EIy4 * invLen;
			float EIz2 = section.EIz2 * invLen;
			float EIy2 = section.EIy2 * invLen;

			//Possible overflow issues with above? Further testing
			/*float EIz12 = 12 * Modulus * Izz / pow(Len, 3);
			float EIy12 = 12 * Modulus * Iyy / pow(Len, 3);
			float EIz6 = 6 * Modulus * Izz / pow(Len, 2);
			float EIy6 = 6 * Modulus * Iyy / pow(Len, 2);
			float EIz4 = 4 * Modulus * Izz / pow(Len ,1);
			float EIy4 = 4 * Modulus * Iyy / pow(Len ,1);
			float EIz2 = 2 * Modulus * Izz / pow(Len ,1);
			float EIy2 = 2 * Modulus * Iyy / pow(Len ,1);*/




			std::vector<Eigen::Triplet<float>> locStiffness_triplets;




			float k1 = Modulus * Area / Len;
			float k2 = section.Ixx * section.G / Len;// FIX THIS THIS IS WRONG <--------------------------------------------------
			locStiffness_triplets.emplace_back(0, 0, k1);
			locStiffness_triplets.emplace_back(0, 6, -k1);
			locStiffness_triplets.emplace_back(1, 1, EIz12);
			locStiffness_triplets.emplace_back(1, 5, EIz6);
			locStiffness_triplets.emplace_back(1, 7, -EIz12);
			locStiffness_triplets.emplace_back(1, 11, EIz6);
			locStiffness_triplets.emplace_back(2, 2, EIy12);
			locStiffness_triplets.emplace_back(2, 4, -EIy6);
			locStiffness_triplets.emplace_back(2, 8, -EIy12);
			locStiffness_triplets.emplace_back(2, 10, -EIy6);
			locStiffness_triplets.emplace_back(3, 3, k2);
			locStiffness_triplets.emplace_back(3, 9, -k2);
			locStiffness_triplets.emplace_back(4, 2, -EIy6);
			locStiffness_triplets.emplace_back(4, 4, EIy4);
			locStiffness_triplets.emplace_back(4, 8, EIy6);
			locStiffness_triplets.emplace_back(4, 10, EIy2);
			locStiffness_triplets.emplace_back(5, 1, EIz6);
			locStiffness_triplets.emplace_back(5, 5, EIz4);
			locStiffness_triplets.emplace_back(5, 7, -EIz6);
			locStiffness_triplets.emplace_back(5, 11, EIz2);
			locStiffness_triplets.emplace_back(6, 0, -k1);
			locStiffness_triplets.emplace_back(6, 6, k1);
			locStiffness_triplets.emplace_back(7, 1, -EIz12);
			locStiffness_triplets.emplace_back(7, 5, -EIz6);
			locStiffness_triplets.emplace_back(7, 7, EIz12);
			locStiffness_triplets.emplace_back(7, 11, -EIz6);
			locStiffness_triplets.emplace_back(8, 2, -EIy12);
			locStiffness_triplets.emplace_back(8, 4, EIy6);
			locStiffness_triplets.emplace_back(8, 8, EIy12);
			locStiffness_triplets.emplace_back(8, 10, EIy6);
			locStiffness_triplets.emplace_back(9, 3, -k2);
			locStiffness_triplets.emplace_back(9, 9, k2);
			locStiffness_triplets.emplace_back(10, 2, -EIy6);
			locStiffness_triplets.emplace_back(10, 4, EIy2);
			locStiffness_triplets.emplace_back(10, 8, EIy6);
			locStiffness_triplets.emplace_back(10, 10, EIy4);
			locStiffness_triplets.emplace_back(11, 1, EIz6);
			locStiffness_triplets.emplace_back(11, 5, EIz2);
			locStiffness_triplets.emplace_back(11, 7, -EIz6);
			locStiffness_triplets.emplace_back(11, 11, EIz4);

			localBmatrix.setFromTriplets(locStiffness_triplets.begin(), locStiffness_triplets.end());
#ifdef DEBUG_PRINTS
			static bool printed = false;
			if (printed)return;
			std::cout << "\n-------------------------------------------------------------------------------\n Local B Matrix\n" << Eigen::MatrixXf(localBmatrix) << "\n";
			printed = true;
#endif
		};


	public:
		size_t node1Pos, node2Pos, node3Pos;


		vBeam(Eigen::Index id_, const Node& N1, const Node& N2, const Node& N3, size_t _sectionId, const Section& section) {
			localBmatrix.resize(12, 12);
			id = id_;
			
			node1Pos = N1.pos;
			node2Pos = N2.pos;
			node3Pos = N3.pos;


			sectionId = _sectionId;
			calc_Len(N2, N1);

			calc_BMatrix(section);
		};

		void reCalc(NodeContainer& Nodes, const Section& section) {
			calc_Len(Nodes.get_byPos(node2Pos), Nodes.get_byPos(node1Pos));
			calc_BMatrix(section);
		}

		const size_t getSectionId() {
			return sectionId;
		}

		Eigen::Index getID() {
			return id;
		}

		void LocalMatrix2GlobalTriplets(std::vector<Eigen::Triplet<float>>& globalTriplets, NodeContainer& Nodes, Section& section) {
			//TODO: refactor stupid get_fromAll function calls. Put this in model
			//using direction cosine matrix because reasons and it works also and no gimbal lock or whatever this stuff is 
			static const Eigen::Vector3d xAxis(1, 0, 0);
			static const Eigen::Vector3d yAxis(0, 1, 0);
			static const Eigen::Vector3d zAxis(0, 0, 1);


			Eigen::Vector3d localX_unit(Nodes.get_byPos(node2Pos).x - Nodes.get_byPos(node1Pos).x, Nodes.get_byPos(node2Pos).y - Nodes.get_byPos(node1Pos).y, Nodes.get_byPos(node2Pos).z - Nodes.get_byPos(node1Pos).z);
			Eigen::Vector3d localY_unit(Nodes.get_byPos(node3Pos).x - Nodes.get_byPos(node1Pos).x, Nodes.get_byPos(node3Pos).y - Nodes.get_byPos(node1Pos).y, Nodes.get_byPos(node3Pos).z - Nodes.get_byPos(node1Pos).z);//a vector in XY local plane
			Eigen::Vector3d localZ_unit;



			localX_unit = localX_unit / localX_unit.norm();
			localY_unit = localY_unit - localY_unit.dot(localX_unit) * localX_unit;// Y local  vector
			localY_unit = localY_unit / localY_unit.norm(); //Y local unit Vector


			localZ_unit = localX_unit.cross(localY_unit);


			std::vector<Eigen::Triplet<float>> dirCosineMat_triplets(37);


			for (size_t i = 0; i < 12; i += 3) {
				dirCosineMat_triplets.emplace_back(0 + i, 0 + i, localX_unit.adjoint() * xAxis);
				dirCosineMat_triplets.emplace_back(0 + i, 1 + i, localX_unit.adjoint() * yAxis);
				dirCosineMat_triplets.emplace_back(0 + i, 2 + i, localX_unit.adjoint() * zAxis);

				dirCosineMat_triplets.emplace_back(1 + i, 0 + i, localY_unit.adjoint() * xAxis);
				dirCosineMat_triplets.emplace_back(1 + i, 1 + i, localY_unit.adjoint() * yAxis);
				dirCosineMat_triplets.emplace_back(1 + i, 2 + i, localY_unit.adjoint() * zAxis);

				dirCosineMat_triplets.emplace_back(2 + i, 0 + i, localZ_unit.adjoint() * xAxis);
				dirCosineMat_triplets.emplace_back(2 + i, 1 + i, localZ_unit.adjoint() * yAxis);
				dirCosineMat_triplets.emplace_back(2 + i, 2 + i, localZ_unit.adjoint() * zAxis);
			}


			Eigen::SparseMatrix<float> cosMatrix(12, 12);
			cosMatrix.setFromTriplets(dirCosineMat_triplets.begin(), dirCosineMat_triplets.end());

			Eigen::SparseMatrix<float> globalB = cosMatrix * localBmatrix * (cosMatrix.transpose());
			//std::cout << Eigen::MatrixXf(globalB);

			Eigen::Index nid1 = Nodes.get_byPos(node1Pos).matrixPos;
			Eigen::Index nid2 = Nodes.get_byPos(node2Pos).matrixPos;

			for (int k = 0; k < 6; ++k)
			{

				Eigen::SparseMatrix<float>::InnerIterator it(globalB, k);
				size_t counter = 0;
				std::ptrdiff_t i;
				for (i = 0; i < 6; ++i) //local Node 1 for Node 1 in global
				{
					Eigen::Index row = (it + i).row(); // row index
					Eigen::Index col = (it + i).col(); // col index (here it is equal to k)
					float val = (it + i).value();

					globalTriplets.emplace_back(nid1 * 6 + row, nid1 * 6 + col, val);

				}
				for (i = 6; it + i; ++i) {
					Eigen::Index row = (it + i).row(); // row index
					Eigen::Index col = (it + i).col(); // col index (here it is equal to k)
					float val = (it + i).value();

					globalTriplets.emplace_back(nid2 * 6 + row - 6, nid1 * 6 + col, val);
				}
			}
			for (int k = 6; k < globalB.outerSize(); ++k)
			{

				Eigen::SparseMatrix<float>::InnerIterator it(globalB, k);
				size_t counter = 0;
				std::ptrdiff_t i;
				for (i = 0; i < 6; ++i) //local Node 1 for Node 1 in global
				{
					Eigen::Index row = (it + i).row(); // row index
					Eigen::Index col = (it + i).col(); // col index (here it is equal to k)
					float val = (it + i).value();



					globalTriplets.emplace_back(nid1 * 6 + row, nid2 * 6 + col - 6, val);

				}
				for (i = 6; it + i; ++i) {
					Eigen::Index row = (it + i).row(); // row index
					Eigen::Index col = (it + i).col(); // col index (here it is equal to k)
					float val = (it + i).value();


					globalTriplets.emplace_back(nid2 * 6 + row - 6, nid2 * 6 + col - 6, val);
				}
			}
		}

	};

	class Model {
		std::vector<Eigen::Triplet<float>> globalK_triplets;
		std::vector<Eigen::Triplet<float>> globalF_triplets;

		Eigen::SparseVector<float> U, F;

		NodeContainer Nodes;

		std::map<size_t,Section> Sections;
		size_t secIdNext = 0;

		std::vector<vBeam> Elements;
		Eigen::Index eId_Last = 0;

		std::vector<size_t> nodes_InMatrixOrder;

		std::set<size_t> BCpinned;//UNUSED- and going to be unused.
		std::set<size_t> BCfixed;
		std::map<size_t, std::array<float, 6>> Forces;//node position to force. Position refers to All nodes, taking into account the deleted stuff.

		size_t noDofs = 0;

		bool solved = false;
		//Raygui does not render large positions well (thigns far away vanish,  probably because I dont use a custom shader). Thus I divide all deflection data /10
		//Done here to only do it once and not every frame.
		Eigen::SparseVector<float> Urender;
		float scaleFactor = 1; //scale Happens Here As Well to do it once and not in every frame
		

		

	public:

		void addNode(Vector3& point) {
			Nodes.emplace(point);
		}

		void addNode(Vector3 point) {
			Nodes.emplace(point);
		}

		//remove node from nth position 
		void removeNode(size_t pos) {
			const Node& node =  Nodes.get_byPos(pos);

			size_t elPos = 0;
			std::set<size_t> toRemove;//Remove from end to beginning. 
			for (auto& element : Elements) {//iterate elements only once. TODO: change with removeElementId when you do binary search
				if (std::find(node.inElements.begin(), node.inElements.end(), element.getID()) != node.inElements.end()) {
					toRemove.insert(elPos);
				}
				elPos++;
			}
			for (auto rit = toRemove.rbegin(); rit != toRemove.rend(); rit++) {
 				removeElement(*rit);
			}

			auto it = Forces.find(pos);
			if (it != Forces.end()) {
				Forces.erase(it);
			}

			auto itB = BCfixed.find(pos);
			if (itB != BCfixed.end()) BCfixed.erase(itB);

			Nodes.remove(pos);	
		}

		bool addElement(size_t n1Pos, size_t n2Pos, size_t n3Pos, size_t sectionID) {

			if (Sections.size() - 1 < sectionID) return false;
			solved = false;


			Elements.emplace_back(eId_Last, Nodes.get_byPos(n1Pos), Nodes.get_byPos(n2Pos), Nodes.get_byPos(n3Pos), sectionID, Sections[sectionID]);
			Sections[sectionID].inElements.emplace_back(eId_Last);

			//get whether the element's nodes were not used in the stifness matrix
			bool n1Free = Nodes.getFree_fromAll(n1Pos);
			bool n2Free = Nodes.getFree_fromAll(n2Pos);


			//ADD DOFs to problem
			noDofs += ((int)n1Free + (int)n2Free) * 6;

			//Set Node positions in the stifness matrix
			if (n1Free) {//it goes in the back of the matrix order
				Nodes.setMatrixPos_fromAll(n1Pos, nodes_InMatrixOrder.size());
				nodes_InMatrixOrder.push_back(n1Pos);
			}
			if (n2Free) {//it goes in the back of the matrix order
				Nodes.setMatrixPos_fromAll(n2Pos, nodes_InMatrixOrder.size());
				nodes_InMatrixOrder.push_back(n2Pos);
			}

			//Update Nodes that are part of Element
			Nodes.add_InElement_fromAll(n1Pos, eId_Last);
			Nodes.add_InElement_fromAll(n2Pos, eId_Last);
			Nodes.add_InElement_fromAll(n3Pos, eId_Last);
			Nodes.setFree_fromAll(n1Pos, false);
			Nodes.setFree_fromAll(n2Pos, false);

			eId_Last++;
			return true;
		}

		bool removeElementId(Eigen::Index eId) {//UNUSED
			//TODO: should do binary search here. Elements are ID sorted (ensure this 100% first)
			size_t counter = 0;
			for (counter = 0; counter < Elements.size(); counter++) {// E ti na kanw re file kakos tropos gia na ta kanw iterate. Binary Search?<----------------------------------;
				vBeam& el = Elements[counter];
				if (el.getID() == eId) {

					Nodes.remove_InElement_fromAll(el.node1Pos,eId);
					Nodes.remove_InElement_fromAll(el.node2Pos,eId);
					Nodes.remove_InElement_fromAll(el.node3Pos,eId);

					
					Elements.erase(Elements.begin() + counter); //menoun ta alla. Tha mporousa na ta kanw iterate kai na riksw ta elID tous... Alla tha eprepe na allaksei to inElements.

					//break;
					solved = false;
					return true;
				}
			}
			return false;
		}

		bool removeElement(size_t ePos) {
			if (ePos >= Elements.size()) return false;

			vBeam& el = Elements[ePos];

			solved = false;

			Eigen::Index eId = el.getID();
			Nodes.remove_InElement_fromAll(el.node1Pos, eId);
			Nodes.remove_InElement_fromAll(el.node2Pos, eId);
			Nodes.remove_InElement_fromAll(el.node3Pos, eId);


			//Housekeeping for node DOFs order. 
			//TODO: Make this more efficient, not passsing twice each time 
			const Node& n1 = Nodes.get_byPos(el.node1Pos);
			const Node& n2 = Nodes.get_byPos(el.node2Pos);

			if (n1.free_flag) {//it became free now
				for (size_t i = n1.matrixPos+1; i < nodes_InMatrixOrder.size(); i++) {
					size_t nextNodePos = nodes_InMatrixOrder[i];
					Nodes.setMatrixPos_fromAll(nextNodePos,Nodes.get_byPos(nextNodePos).matrixPos-1); //for the next Nodes (in matrix order) reduce the matrixPosition by 1
				}
				noDofs -= 6;
				nodes_InMatrixOrder.erase(nodes_InMatrixOrder.begin() + n1.matrixPos);
				Nodes.setMatrixPos_fromAll(n1.pos, -1);
			}
			if (n2.free_flag) {//it became free now
				for (size_t i = n2.matrixPos + 1; i < nodes_InMatrixOrder.size(); i++) {
					size_t nextNodePos = nodes_InMatrixOrder[i];
					Nodes.setMatrixPos_fromAll(nextNodePos, Nodes.get_byPos(nextNodePos).matrixPos - 1); //for the next Nodes (in matrix order) reduce the matrixPosition by 1
				}
				noDofs -= 6;
				nodes_InMatrixOrder.erase(nodes_InMatrixOrder.begin() + n2.matrixPos);
				Nodes.setMatrixPos_fromAll(n2.pos, -1);

			}



			Elements.erase(Elements.begin() + ePos); 

			return true;


		}

		void oneElementTest() {

			addNode(Vector3{ 0,0,0 });
			addNode(Vector3{ 50, 0, 0 });
			addNode(Vector3{ 0.2, 50,0 });
			addNode(Vector3{ 100, 0, 0 });
			addNode(Vector3{ 150, 0, 0 });
			addNode(Vector3{ 200, 0, 0 });
			addNode(Vector3{ 25, 50, 0 });
			addNode(Vector3{ 75, 50, 0 });
			addNode(Vector3{ 125, 50, 0 });


			addSection(100, 210000, 80000, 1000, 100, 100);

			addElement(0, 1, 2, 0);
			addElement(1, 3, 2, 0);
			addElement(3, 4, 2, 0);
			addElement(4, 5, 2, 0);


			BCfixed.emplace(0);
			Forces[4][1] = 100;

		};

		void addSection(float _Area, float _Modulus, float _G, float _Ixx, float _Iyy, float _Izz) {
			Sections.emplace(std::make_pair(secIdNext, Section{ _Area, _Modulus, _G, _Ixx, _Iyy, _Izz }));
			secIdNext++;
		}

		//nodePos is the position containing deleted nodes. 
		void addForce(size_t nodePos, size_t Dof, float val) {
			std::array<float,6> ar{ 0,0,0,0,0,0 };
			auto it = Forces.emplace(std::make_pair(nodePos, ar));
			it.first->second[Dof] = val;
			solved = false;

		}

		void solve() {

			//----------------------------------------------------------------------------------------------------
			//Setup 
			//----------------------------------------------------------------------------------------------------
			//TODO: Check for unconstrained model
			
			globalK_triplets.clear();
			globalF_triplets.clear();
			if (BCfixed.size() + BCpinned.size() < 1) {
				solved = false;
				return;
			}
			if (noDofs < 1) return;
			
			#ifdef DEBUG_PRINTS
				std::cout << "\n------------------------------------\nNO DOFs: " << noDofs << "\n";
			#endif // DEBUG_PRINTS
			


			//----------------------------------------------------------------------------------------------------
			//Populate global stigness matrix (triplets) from each element
			//----------------------------------------------------------------------------------------------------
			#ifdef DEBUG_PRINTS
				Eigen::SparseMatrix<float> testGlobAll(noDofs, noDofs);
			#endif

			for (auto& element : Elements) {
				element.LocalMatrix2GlobalTriplets(globalK_triplets, Nodes, Sections[element.getSectionId()]);

			#ifdef DEBUG_PRINTS
				testGlobAll.setFromTriplets(globalK_triplets.begin(), globalK_triplets.end());
			#endif // DEBUG_PRINTS

			}

			#ifdef DEBUG_PRINTS
				std::cout << "\n------------------------------------\n Glob Matrix noRowDeletion:\n " << Eigen::MatrixXf(testGlobAll) << "\n";
			#endif // DEBUG_PRINTS

			size_t noDofsUsed = noDofs- 6 * (BCfixed.size()) - 3 * BCpinned.size();
			Eigen::SparseMatrix<float> globMatr(noDofsUsed, noDofsUsed);



			//----------------------------------------------------------------------------------------------------
			//Stifness Marix Row/Column Elimination from BCs & create Global K Matrix
			//----------------------------------------------------------------------------------------------------
			//TODO: Better solution without triplet duplication. (faster?)
			
			std::vector<Eigen::Triplet<float>> triplets_AfterBCs;
			std::vector<Eigen::Triplet<float>> triplets_AfterBCsALL;
			int row, col, val;
			int minusRow, minusCol;
			

			for (auto& triplet : globalK_triplets) {
				row = triplet.row(); col = triplet.col(); val = triplet.value();
				minusRow = 0; minusCol = 0; 
				bool isInBC = false;

				for (size_t fixBCid : BCfixed) {
					size_t BCstart = Nodes.get_byPos(fixBCid).matrixPos * 6;
					size_t BCend = Nodes.get_byPos(fixBCid).matrixPos * 6 + 5;
					const static size_t BCdofs = 6;

					if (row > BCend) minusRow += BCdofs;//if after BC, row is BC dofs less.(row elimination.)
					else if(row >= BCstart){ //not after BC end dof but after BC start dof means in BC dofs
						isInBC = true;
						break; // go to next, no matter the column
					}

					if (col > BCend) minusCol+= BCdofs;
					else if (col >= BCstart) {
						isInBC = true;
						break;
					}
					
				}
				if(!isInBC) triplets_AfterBCs.emplace_back(row-minusRow, col-minusCol, val);
			
			//#ifdef DEBUG_PRINTS
				if (!isInBC) triplets_AfterBCsALL.emplace_back(row, col, val);
				else triplets_AfterBCsALL.emplace_back(row, col, -101);
			//#endif // DEBUG_PRINTS
			
			}

			globMatr.setFromTriplets(triplets_AfterBCs.begin(), triplets_AfterBCs.end());
			
			#ifdef DEBUG_PRINTS
				Eigen::SparseMatrix<float> asd(noDofs, noDofs);
				asd.setFromTriplets(triplets_AfterBCsALL.begin(), triplets_AfterBCsALL.end());
				std::cout << "\n------------------------------------\n Glob Matrix with noted removed Rows/Colsn\n " << Eigen::MatrixXf(asd) << "\n";
				std::cout << "\n------------------------------------\n Glob Matrix after row elimination\n " << Eigen::MatrixXf(globMatr) << "\n";

			#endif // DEBUG_PRINTS
			


			//----------------------------------------------------------------------------------------------------
			//F Vector Creation and Row Elimination
			//----------------------------------------------------------------------------------------------------
			
			F.resize(globMatr.rows());

			std::vector<Eigen::Triplet<float>> Ftriplets_AfterBCs;
			//make forces vector 
			for (auto& force : Forces) {
				size_t forceMatrixPos = Nodes.get_byPos(force.first).matrixPos*6;
				bool inBC = false;

				for (size_t BCid : BCfixed) {
					size_t BCstart = Nodes.get_byPos(BCid).matrixPos;
					size_t BCend = Nodes.get_byPos(BCid).matrixPos + 5;
					if (forceMatrixPos > BCend) forceMatrixPos -= 6;
					else if (forceMatrixPos >= BCstart) {
						inBC = true;
						break;
					}

				}
				if (inBC || forceMatrixPos >= F.rows()) continue;

				F.insert((Eigen::Index)forceMatrixPos) = force.second[0];
				F.insert((Eigen::Index)forceMatrixPos + 1) = force.second[1];
				F.insert((Eigen::Index)forceMatrixPos + 2) = force.second[2];
				F.insert((Eigen::Index)forceMatrixPos + 3) = force.second[3];
				F.insert((Eigen::Index)forceMatrixPos + 4) = force.second[4];
				F.insert((Eigen::Index)forceMatrixPos + 5) = force.second[5];

			}
			#ifdef DEBUG_PRINTS
				std::cout << "\n--------------------------------\nForce Vector\n" << F << "\n";
			#endif // DEBUG_PRINTS



			//----------------------------------------------------------------------------------------------------
			//Solving
			//----------------------------------------------------------------------------------------------------
			//TODO: Proper handling of failed solves.

			Eigen::SparseLU<Eigen::SparseMatrix<float>> solver;
			solver.compute(globMatr);

			if (solver.info() == Eigen::Success) {
				// Decomposition Succesfull
				std::cout << "Sparse LU Decomposition Successful\n";
				U = solver.solve(F);
				if (solver.info() != Eigen::Success) {
					// solving failed
					std::cout << "solving failed - Exiting";
					return;
				}
				std::cout << "Sparse Solving Successful\n";
				solved = true;
				std::cout << U;
				Urender = U * 0.1 * scaleFactor;

				return;
			}


			std::cout << "Sparse LU decomposition failed\nConverting to Dense\n";
			Eigen::MatrixXf A(globMatr);// DO EXCEPTION HERE if this fails Also
			U = A.lu().solve(Eigen::VectorXf(F)).sparseView();
			std::cout << F << "\n" << A << "\n" << U;
			Urender.resize(U.rows());
			Urender = U * RENDER_SCALING_FACTOR * scaleFactor;
			solved = true;

		}

		const NodeContainer& getNodes() {
			return Nodes;
		}

		const std::vector<vBeam>& getElements() {
			return Elements;
		}

		Vector3 getDeflection(size_t nodeMatrixPos) {
			if (!solved) return Vector3Zero();

			size_t fixedNodesBefore = 0;
			for (size_t fixedId : BCfixed) {
				size_t BCmatrixPos = Nodes.get_byPos(fixedId).matrixPos;
				if (BCmatrixPos < nodeMatrixPos) fixedNodesBefore++;
				else if (BCmatrixPos == nodeMatrixPos) return Vector3Zero();
			}
			size_t afterBcId = nodeMatrixPos * 6 - fixedNodesBefore * 6;
			if (afterBcId >= U.rows()) return Vector3Zero(); //Free node, not in Stifness matrix


			return Vector3{ U.coeff(afterBcId),U.coeff(afterBcId + 1),U.coeff(afterBcId + 2) };

		}

		Vector3 getDeflectionRender(size_t nodeMatrixPos) {//doesn't give rotations. 
			if (!solved) return Vector3Zero();

			if (nodeMatrixPos >= nodes_InMatrixOrder.size()) return Vector3Zero();//not in stifness matrix
			const Node& node = Nodes.get_byPos(nodes_InMatrixOrder[nodeMatrixPos]);

			if (node.free_flag) return Vector3Zero();

			size_t fixedNodesBefore = 0;
			for (size_t fixedId : BCfixed) {
				size_t BCmatrixPos = Nodes.get_byPos(fixedId).matrixPos;
				if (BCmatrixPos < nodeMatrixPos) fixedNodesBefore++;
				else if (BCmatrixPos == nodeMatrixPos) return Vector3Zero();
			}

			size_t afterBcId = nodeMatrixPos * 6 - fixedNodesBefore * 6;

			return Vector3{ Urender.coeff(afterBcId) ,Urender.coeff(afterBcId + 1) ,Urender.coeff(afterBcId + 2) };

		}

		Vector3 getForce(size_t nodePos) {
			auto it = Forces.find(nodePos);

			if (it == Forces.end()) return Vector3Zero();
			return Vector3{ it->second[0],it->second[1],it->second[2] };
		}

		void removeForce(size_t nodePos) {
			auto it = Forces.erase(nodePos);
			solved = false;

		}

		const std::map<size_t, std::array<float, 6>>& getForces() {
			return Forces;
		}

		void addBCfixed(size_t nodePos) {
			solved = false;
			if (Nodes.get_byPos(nodePos).free_flag) return;
			BCfixed.emplace(nodePos);
		}

		void removeBCfixed(size_t nodePos) {
			BCfixed.erase(nodePos);
			solved = false;

		}

		const std::set<size_t>& getBCfixed() {
			return BCfixed;
		}

		void printDeformed() {
			for (auto& node : Nodes) {
			
				Vector3 a;
				if (node.free_flag) {
					a = Vector3Zero();
					continue;
				}
				a = getDeflection(node.matrixPos);
				std::cout << a.x << " " << a.y << " " << a.z << " \n";
			}

		}

		void printU() {
			std::cout << Eigen::VectorXf{ U } << "\n";
		}

		void printF() {
			std::cout << Eigen::VectorXf{ F } << "\n";
		}

		bool isSolved() {
			return solved;
		}

		const std::map<size_t,Section>& getSections() {
			return Sections;
		}

		void modifySection(const size_t Id, float _Area, float _Modulus, float _G, float _Ixx, float _Iyy, float _Izz) {
			auto it = Sections.find(Id);

			if (it != Sections.end()) {
				solved = false;
				Section& sec = it->second;
				
				sec.Area = _Area;
				sec.Izz = (_Izz < 0) ? 100 : _Izz;
				sec.Ixx = (_Ixx < 0) ? 100 : _Ixx;
				sec.Iyy = (_Iyy < 0) ? _Izz : _Iyy;
				sec.Modulus = _Modulus;
				sec.G = _G;
				for (auto& element : Elements) {
					if (std::find(sec.inElements.begin(), sec.inElements.end(), element.getID()) != sec.inElements.end()) {
						element.reCalc(Nodes, sec);
					}
				}
			}
		}
	};
};



