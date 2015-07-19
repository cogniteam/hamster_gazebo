#ifndef __COGNI_FUZZY_LOGIC___H__
#define __COGNI_FUZZY_LOGIC___H__

#include <iostream>
using namespace std;

#include <vector>
#include <set>
#include <map>
#include <sstream>
#include <math.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/foreach.hpp>
#include <boost/bind.hpp>
using namespace boost::geometry;
typedef boost::geometry::model::d2::point_xy<double> point_2d;
typedef boost::geometry::model::polygon<point_2d> polygon_2d;
typedef boost::geometry::model::box<point_2d> box_2d;
typedef vector<polygon_2d> polygon_list;

#define DEBUG(X) //cout<<X<<endl;

class Function{
public:
	typedef boost::geometry::model::d2::point_xy<double> point_2d;
	typedef boost::geometry::model::polygon<point_2d> polygon_2d;
	typedef boost::geometry::model::box<point_2d> box_2d;
	typedef vector<polygon_2d> polygon_list;
	typedef boost::geometry::model::linestring<point_2d> path_2d;

	polygon_2d points;
	point_2d maxs;
	point_2d mins;
	polygon_list list;
	Function(double x, double y){
		maxs = point_2d(x,y);
		mins = point_2d(x,y);
		add(x,y);
	}
	Function(const polygon_2d& pol){
		maxs = point_2d(pol.outer().front().x(),pol.outer().front().y());
		mins = maxs;
		points = pol;
		BOOST_FOREACH(point_2d p, points.outer()){
			maxs = point_2d(fmax(maxs.x(),p.x()),fmax(maxs.y(),p.y()));
			mins = point_2d(fmin(mins.x(),p.x()),fmin(mins.y(),p.y()));
		}
	}
	Function(const polygon_list& plist){
		maxs = point_2d(plist.front().outer().front().x(),plist.front().outer().front().y());
		mins = maxs;
		list = plist;
		BOOST_FOREACH(polygon_2d points, plist){
			this->points = points;
			BOOST_FOREACH(point_2d p, points.outer()){
				maxs = point_2d(fmax(maxs.x(),p.x()),fmax(maxs.y(),p.y()));
				mins = point_2d(fmin(mins.x(),p.x()),fmin(mins.y(),p.y()));
			}
		}
	}
	Function& operator()(double x, double y){
		add(x, y); return *this;
	}
	void add(double x, double y){
		maxs = point_2d(fmax(maxs.x(),x),fmax(maxs.y(),y));
		mins = point_2d(fmin(mins.x(),x),fmin(mins.y(),y));
		if(points.outer().size()==0 and y>0) add(x, 0);
		boost::geometry::append(points.outer(), point_2d(x,y));
	}
	Function& end(){
		if(points.outer().size()>0 and points.outer().back().y()>0) add(points.outer().back().x(),0);
		//boost::geometry::correct(points);
		polygon_list tmp;
		box_2d b(point_2d(mins.x(),0), point_2d(maxs.x(), 1));
		boost::geometry::intersection(points,b,tmp);
		if(tmp.size()==0) list.push_back(polygon_2d());
		else list.push_back(tmp.front());
		return *this;
	}
	double y(double x)const{
		if(x<mins.x() or x>maxs.x()) return 0;
		path_2d p1, p2, tmp;
		boost::geometry::append(p1, point_2d(x,0));
		boost::geometry::append(p1, point_2d(x,maxs.y()));
		BOOST_FOREACH(polygon_2d pol, list){
			boost::geometry::intersection(p1, pol, tmp);
			size_t s = p2.size(); p2.resize(p2.size()+tmp.size());
			copy(tmp.begin(), tmp.end(), p2.begin()+s);
		}
		BOOST_FOREACH(point_2d p, p2) if(p.y()>0) return p.y();
		return 0;
	}
	std::vector<double> x(double y)const{
		std::vector<double> res;
		path_2d p1, p2, tmp;
		boost::geometry::append(p1, point_2d(mins.x()-1,y));
		boost::geometry::append(p1, point_2d(maxs.x()+1,y));
		BOOST_FOREACH(polygon_2d pol, list){
			boost::geometry::intersection(p1, pol, tmp);
			size_t s = p2.size(); p2.resize(p2.size()+tmp.size());
			copy(tmp.begin(), tmp.end(), p2.begin()+s);
		}
		BOOST_FOREACH(point_2d p, p2) res.push_back( p.x() );
		return res;
	}
	double area()const{
		double a=0;
		BOOST_FOREACH(polygon_2d pol, list){
			a += boost::geometry::area(pol);
		}
		return a;
	}
	point_2d centroid()const{
		point_2d cen;
		double aa=0;
		BOOST_FOREACH(polygon_2d pol, list){
			point_2d c; double a;
			a = boost::geometry::area(pol);
			if(a==0) continue;
			boost::geometry::centroid(pol,c);
			cen.x(cen.x()+c.x()*a); cen.y(cen.y()+c.y()*a);
			aa+=a;
		}
		if(aa==0){ DEBUG("ERROR: can't calculate centroid of empty set."); return point_2d(0,0); }
		return point_2d( cen.x()/aa, cen.y()/aa );
	}
	Function Union(const Function& f)const{
		polygon_list res;
		BOOST_FOREACH(polygon_2d p1, list){BOOST_FOREACH(polygon_2d p2, f.list){
			polygon_list tmp;
			DEBUG("union: "<<dsv(p1)<<" -- "<<dsv(p2));
			boost::geometry::union_(p1,p2,tmp);
			BOOST_FOREACH(polygon_2d pt, tmp) DEBUG("RES: "<<dsv(pt));
			size_t s = res.size(); res.resize(res.size()+tmp.size());
			copy(tmp.begin(), tmp.end(), res.begin()+s);
		}}
		if(res.size()==0){ DEBUG("res: "<<dsv(polygon_2d())); return Function(0,0).end(); }
		return Function(res);
	}
	Function Intersect(const Function& f)const{
		polygon_list res;
		BOOST_FOREACH(polygon_2d p1, list){BOOST_FOREACH(polygon_2d p2, f.list){
			polygon_list tmp;
			DEBUG("intersect: "<<dsv(p1)<<" -- "<<dsv(p2));
			boost::geometry::intersection(p1,p2,tmp);
			size_t s = res.size(); res.resize(res.size()+tmp.size());
			copy(tmp.begin(), tmp.end(), res.begin()+s);
		}}
		if(res.size()==0){ DEBUG("res: "<<dsv(polygon_2d())); return Function(0,0).end(); }
		return Function(res);
	}
	Function cut(double y)const{
		polygon_list res;
		box_2d b(point_2d(mins.x()-1,0), point_2d(maxs.x()+1, y));
		BOOST_FOREACH(polygon_2d p1, list){
			polygon_list tmp;
			DEBUG("cut: "<<dsv(p1)<<" -- "<<dsv(b));
			boost::geometry::intersection(p1,b,tmp);
			BOOST_FOREACH(polygon_2d pt, tmp) DEBUG("RES: "<<dsv(pt));
			size_t s = res.size(); res.resize(res.size()+tmp.size());
			copy(tmp.begin(), tmp.end(), res.begin()+s);
		}
		if(res.size()==0){ DEBUG("res: "<<dsv(polygon_2d())); return Function(0,0).end(); }
		return Function(res);
	}
	template <class A>
	Function apply(A a)const{
		polygon_list res;
		BOOST_FOREACH(polygon_2d p1, list){
			polygon_2d p2;
			BOOST_FOREACH(point_2d p, p1.outer()){
				p2.outer().push_back(point_2d(p.x(),a(p.y())));
			}
			res.push_back(p2);
		}
		return res;
	}

	std::string str()const{
		stringstream s;
		s.precision(2);
		s.setf( std::ios::fixed, std:: ios::floatfield );
		s<<"Function: "<<endl;
		BOOST_FOREACH(polygon_2d pol, list){
			s << " "<< boost::geometry::dsv(pol) << endl;
		}
		s<<"---";
		return s.str();
	}
};

struct FDegree{
	double value;
	operator double()const{ return value; }
	const FDegree& operator=(double v){ value = v; return *this; }
	FDegree(double v=0):value(v){}
	FDegree(const FDegree& v):value(v.value){}
};

class FSet{
	Function f;
public:
	std::string id;
	FSet(const Function& f,std::string id):f(f),id(id){}
	virtual ~FSet(){}
	virtual FDegree degree(double v)const{ return FDegree(f.y(v)); }
	virtual double center(FDegree degree)const{ return function().cut(degree.value).centroid().x(); }
	virtual const Function& function()const{ return f; }
	FDegree operator*(double v)const{ return degree(v); }
};


template<class SET>
class FValue{
public:
	FValue(SET* set, FDegree d):set(set),degree(d){}
	template<class SET1>
	FValue(const FValue<SET1>& other):set((SET*)other.set),degree(other.degree){}
	SET* set;
	FDegree degree;
	operator FDegree()const{ return degree; }
	Function f()const{ return set->function().cut(degree.value); }
	std::string id()const{ return set->id; }
};

FDegree operator&&(FDegree x, FDegree y){ return fmin(x.value,y.value); }
FDegree operator||(FDegree x, FDegree y){ return fmax(x.value,y.value); }
FDegree operator!(FDegree x){ return 1-x.value; }

template <class SET>
class FVariable{
public:
	vector< FValue<SET> > sets;
	FVariable<SET>& push(const FValue<SET>& set){ sets.push_back(set); return *this; }
	FVariable<SET>& operator=(const FValue<SET>& set){ return push(set); }
	FDegree degree()const{
		if(sets.empty()) return 0;
		FDegree d = sets[0];
		for(size_t i=1;i<sets.size();i++) d = d and sets[i];
		return d;
	}
	operator FDegree()const{ return degree(); }
	const FDegree operator==(const SET& set)const{
		for(size_t i=0;i<sets.size();i++){
			if(sets[i].id() == set.id) return sets[i].degree;
		}
		return FDegree(0);
	}

	double defuzzification()const{
		Function all = sets[0].f();
		for(size_t i=1;i<sets.size();i++){
			all = all.Union( sets[i].f() );
		}
		return all.centroid().x();
	}

};

const FDegree operator==(double value, const FSet& set){
	return set.degree(value);
}
const FDegree operator==(const FSet& set, double value){
	return set.degree(value);
}

template<class FSet>
const FValue<FSet> operator<=(FSet& set, FDegree x){ FValue<FSet> nset = FValue<FSet>(&set,x); return nset; }

#define DEF_TYPE(NAME)\
	class NAME: public FSet{public:\
		typedef FVariable<NAME> Variable;\
		NAME(const Function& f, std::string id):FSet(f,id){}\
		NAME(std::string id):FSet(Function(0,0),id){}\
	};
#define DEF_SET(NAME, TYPE, ... )\
	class NAME##_set:public TYPE{\
	public:\
		NAME##_set():TYPE(Function __VA_ARGS__ .end(), "/"#NAME){}\
	} NAME;
#define DEF_SEMANTICS(NAME,SET,...)\
	class NAME##_TYPE{\
	public:\
		class NEW_TERM: public SET{\
		public:\
			mutable Function f_tmp;\
			const SET& set;\
			NEW_TERM(const SET& set):SET(set.id+("/"#NAME)),f_tmp(0,0),set(set){}\
			double _degree(double v)const{ __VA_ARGS__ }\
			FDegree degree(double v)const{ return _degree(v); }\
			const Function& function()const{ f_tmp = set.function().apply(boost::bind(&NAME##_TYPE::NEW_TERM::_degree, this,_1)); return f_tmp; }\
		};\
		const NEW_TERM build(const SET& set)const{ return NEW_TERM(set); }\
		const NEW_TERM operator*(const SET& set)const{ return build(set); }\
	} NAME;

#define is ==


#endif
