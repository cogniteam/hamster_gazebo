/*******************************************************************************
*                                                                              *
* Author    :  Angus Johnson                                                   *
* Version   :  5.1.5                                                           *
* Date      :  4 May 2013                                                      *
* Website   :  http://www.angusj.com                                           *
* Copyright :  Angus Johnson 2010-2013                                         *
*                                                                              *
* License:                                                                     *
* Use, modification & distribution is subject to Boost Software License Ver 1. *
* http://www.boost.org/LICENSE_1_0.txt                                         *
*                                                                              *
* Attributions:                                                                *
* The code in this library is an extension of Bala Vatti's clipping algorithm: *
* "A generic solution to polygon clipping"                                     *
* Communications of the ACM, Vol 35, Issue 7 (July 1992) pp 56-63.             *
* http://portal.acm.org/citation.cfm?id=129906                                 *
*                                                                              *
* Computer graphics and geometric modeling: implementation and algorithms      *
* By Max K. Agoston                                                            *
* Springer; 1 edition (January 4, 2005)                                        *
* http://books.google.com/books?q=vatti+clipping+agoston                       *
*                                                                              *
* See also:                                                                    *
* "Polygon Offsetting by Computing Winding Numbers"                            *
* Paper no. DETC2005-85513 pp. 565-575                                         *
* ASME 2005 International Design Engineering Technical Conferences             *
* and Computers and Information in Engineering Conference (IDETC/CIE2005)      *
* September 24-28, 2005 , Long Beach, California, USA                          *
* http://www.me.berkeley.edu/~mcmains/pubs/DAC05OffsetPolygon.pdf              *
*                                                                              *
*******************************************************************************/

#ifndef clipper_hpp
#define clipper_hpp

#include <vector>
#include <stdexcept>
#include <cstring>
#include <cstdlib>
#include <ostream>

namespace poly {

enum ClipType { ctIntersection, ctUnion, ctDifference, ctXor };
enum PolyType { ptSubject, ptClip };
//By far the most widely used winding rules for polygon filling are
//EvenOdd & NonZero (GDI, GDI+, XLib, OpenGL, Cairo, AGG, Quartz, SVG, Gr32)
//Others rules include Positive, Negative and ABS_GTR_EQ_TWO (only in OpenGL)
//see http://glprogramming.com/red/chapter11.html
enum PolyFillType { pftEvenOdd, pftNonZero, pftPositive, pftNegative };

typedef signed long long long64;
typedef unsigned long long ulong64;

struct IntPoint {
public:
  long64 X;
  long64 Y;
  IntPoint(long64 x = 0, long64 y = 0): X(x), Y(y) {};
  friend std::ostream& operator <<(std::ostream &s, const IntPoint &p);
};

inline bool PointsEqual( const IntPoint &pt1, const IntPoint &pt2)
{
  return ( pt1.X == pt2.X && pt1.Y == pt2.Y );
}
//------------------------------------------------------------------------------


typedef std::vector< IntPoint > Polygon;
typedef std::vector< Polygon > Polygons;


std::ostream& operator <<(std::ostream &s, const Polygon &p);
std::ostream& operator <<(std::ostream &s, const Polygons &p);

template<class T>
std::vector<T>& operator<<(std::vector<T>& vec, const T& v){ vec.push_back(v); return vec; }

inline
Polygons& operator<<(Polygons& vec, const IntPoint& v){ if(vec.empty()) vec.push_back(Polygon()); vec.back().push_back(v); return vec; }
inline
Polygons& operator<<(Polygons& vec, const Polygons& v){ for(size_t i=0;i<v.size();i++)vec.push_back(v[i]); return vec; }

class PolyNode;
typedef std::vector< PolyNode* > PolyNodes;

class PolyNode 
{ 
public:
    PolyNode();
    Polygon Contour;
    PolyNodes Childs;
    PolyNode* Parent;
    PolyNode* GetNext() const;
    bool IsHole() const;
    int ChildCount() const;
private:
    PolyNode* GetNextSiblingUp() const;
    unsigned Index; //node index in Parent.Childs
    void AddChild(PolyNode& child);
    friend class Clipper; //to access Index
};

class PolyTree: public PolyNode
{ 
public:
    ~PolyTree(){Clear();};
    PolyNode* GetFirst() const;
    void Clear();
    int Total() const;
private:
    PolyNodes AllNodes;
    friend class Clipper; //to access AllNodes
    friend class NodesIterator;
};

class NodesIterator{
	const PolyTree& tree;
public:
	NodesIterator(const PolyTree& t):tree(t){}
	PolyNode* operator[](size_t i)const{ return tree.AllNodes[i]; }
	size_t size()const{ return tree.AllNodes.size(); }
};
        
enum JoinType { jtSquare, jtRound, jtMiter };

bool Orientation(const Polygon &poly);
double Area(const Polygon &poly);
double Area(const PolyTree &poly);

void OffsetPolygons(const Polygons &in_polys, Polygons &out_polys,
  double delta, JoinType jointype = jtSquare, double limit = 0, bool autoFix = true);

void SimplifyPolygon(const Polygon &in_poly, Polygons &out_polys, PolyFillType fillType = pftEvenOdd);
void SimplifyPolygons(const Polygons &in_polys, Polygons &out_polys, PolyFillType fillType = pftEvenOdd);
void SimplifyPolygons(Polygons &polys, PolyFillType fillType = pftEvenOdd);

void CleanPolygon(Polygon& in_poly, Polygon& out_poly, double distance = 1.415);
void CleanPolygons(Polygons& in_polys, Polygons& out_polys, double distance = 1.415);

void PolyTreeToPolygons(const PolyTree& polytree, Polygons& polygons);

void ReversePolygon(Polygon& p);
void ReversePolygons(Polygons& p);

//used internally ...
enum EdgeSide { esLeft = 1, esRight = 2};
enum IntersectProtects { ipNone = 0, ipLeft = 1, ipRight = 2, ipBoth = 3 };

struct TEdge {
  long64 xbot;
  long64 ybot;
  long64 xcurr;
  long64 ycurr;
  long64 xtop;
  long64 ytop;
  double dx;
  long64 deltaX;
  long64 deltaY;
  PolyType polyType;
  EdgeSide side;
  int windDelta; //1 or -1 depending on winding direction
  int windCnt;
  int windCnt2; //winding count of the opposite polytype
  int outIdx;
  TEdge *next;
  TEdge *prev;
  TEdge *nextInLML;
  TEdge *nextInAEL;
  TEdge *prevInAEL;
  TEdge *nextInSEL;
  TEdge *prevInSEL;
};

struct IntersectNode {
  TEdge          *edge1;
  TEdge          *edge2;
  IntPoint        pt;
  IntersectNode  *next;
};

struct LocalMinima {
  long64        Y;
  TEdge        *leftBound;
  TEdge        *rightBound;
  LocalMinima  *next;
};

struct Scanbeam {
  long64    Y;
  Scanbeam *next;
};

struct OutPt; //forward declaration

struct OutRec {
  int     idx;
  bool    isHole;
  OutRec *FirstLeft;  //see comments in clipper.pas
  PolyNode *polyNode;
  OutPt  *pts;
  OutPt  *bottomPt;
};

struct OutPt {
  int     idx;
  IntPoint pt;
  OutPt   *next;
  OutPt   *prev;
};

struct JoinRec {
  IntPoint  pt1a;
  IntPoint  pt1b;
  int       poly1Idx;
  IntPoint  pt2a;
  IntPoint  pt2b;
  int       poly2Idx;
};

struct HorzJoinRec {
  TEdge    *edge;
  int       savedIdx;
};

struct IntRect { long64 left; long64 top; long64 right; long64 bottom; };

typedef std::vector < OutRec* > PolyOutList;
typedef std::vector < TEdge* > EdgeList;
typedef std::vector < JoinRec* > JoinList;
typedef std::vector < HorzJoinRec* > HorzJoinList;

//ClipperBase is the ancestor to the Clipper class. It should not be
//instantiated directly. This class simply abstracts the conversion of sets of
//polygon coordinates into edge objects that are stored in a LocalMinima list.
class ClipperBase
{
public:
  ClipperBase();
  virtual ~ClipperBase();
  bool AddPolygon(const Polygon &pg, PolyType polyType);
  bool AddPolygons( const Polygons &ppg, PolyType polyType);
  virtual void Clear();
  IntRect GetBounds();
protected:
  void DisposeLocalMinimaList();
  TEdge* AddBoundsToLML(TEdge *e);
  void PopLocalMinima();
  virtual void Reset();
  void InsertLocalMinima(LocalMinima *newLm);
  LocalMinima      *m_CurrentLM;
  LocalMinima      *m_MinimaList;
  bool              m_UseFullRange;
  EdgeList          m_edges;
};

class Clipper : public virtual ClipperBase
{
public:
  Clipper();
  ~Clipper();
  bool Execute(ClipType clipType,
    Polygons &solution,
    PolyFillType subjFillType = pftEvenOdd,
    PolyFillType clipFillType = pftEvenOdd);
  bool Execute(ClipType clipType,
    PolyTree &polytree,
    PolyFillType subjFillType = pftEvenOdd,
    PolyFillType clipFillType = pftEvenOdd);
  void Clear();
  bool ReverseSolution() {return m_ReverseOutput;};
  void ReverseSolution(bool value) {m_ReverseOutput = value;};
  bool ForceSimple() {return m_ForceSimple;};
  void ForceSimple(bool value) {m_ForceSimple = value;};
  OutPt* getFistOutPt()const{if(m_PolyOuts.size()>0)return m_PolyOuts[0]->pts;return NULL;}
protected:
  void Reset();
  virtual bool ExecuteInternal();
private:
  PolyOutList       m_PolyOuts;
  JoinList          m_Joins;
  HorzJoinList      m_HorizJoins;
  ClipType          m_ClipType;
  Scanbeam         *m_Scanbeam;
  TEdge           *m_ActiveEdges;
  TEdge           *m_SortedEdges;
  IntersectNode   *m_IntersectNodes;
  bool             m_ExecuteLocked;
  PolyFillType     m_ClipFillType;
  PolyFillType     m_SubjFillType;
  bool             m_ReverseOutput;
  bool             m_UsingPolyTree; 
  bool             m_ForceSimple;
  void DisposeScanbeamList();
  void SetWindingCount(TEdge& edge);
  bool IsEvenOddFillType(const TEdge& edge) const;
  bool IsEvenOddAltFillType(const TEdge& edge) const;
  void InsertScanbeam(const long64 Y);
  long64 PopScanbeam();
  void InsertLocalMinimaIntoAEL(const long64 botY);
  void InsertEdgeIntoAEL(TEdge *edge);
  void AddEdgeToSEL(TEdge *edge);
  void CopyAELToSEL();
  void DeleteFromSEL(TEdge *e);
  void DeleteFromAEL(TEdge *e);
  void UpdateEdgeIntoAEL(TEdge *&e);
  void SwapPositionsInSEL(TEdge *edge1, TEdge *edge2);
  bool IsContributing(const TEdge& edge) const;
  bool IsTopHorz(const long64 XPos);
  void SwapPositionsInAEL(TEdge *edge1, TEdge *edge2);
  void DoMaxima(TEdge *e, long64 topY);
  void ProcessHorizontals();
  void ProcessHorizontal(TEdge *horzEdge);
  void AddLocalMaxPoly(TEdge *e1, TEdge *e2, const IntPoint &pt);
  void AddLocalMinPoly(TEdge *e1, TEdge *e2, const IntPoint &pt);
  OutRec* GetOutRec(int idx);
  void AppendPolygon(TEdge *e1, TEdge *e2);
  void IntersectEdges(TEdge *e1, TEdge *e2,
    const IntPoint &pt, const IntersectProtects protects);
  OutRec* CreateOutRec();
  void AddOutPt(TEdge *e, const IntPoint &pt);
  void DisposeAllPolyPts();
  void DisposeOutRec(PolyOutList::size_type index);
  bool ProcessIntersections(const long64 botY, const long64 topY);
  void AddIntersectNode(TEdge *e1, TEdge *e2, const IntPoint &pt);
  void BuildIntersectList(const long64 botY, const long64 topY);
  void ProcessIntersectList();
  void ProcessEdgesAtTopOfScanbeam(const long64 topY);
  void BuildResult(Polygons& polys);
  void BuildResult2(PolyTree& polytree);
  void SetHoleState(TEdge *e, OutRec *outrec);
  void DisposeIntersectNodes();
  bool FixupIntersectionOrder();
  void FixupOutPolygon(OutRec &outrec);
  bool IsHole(TEdge *e);
  void FixHoleLinkage(OutRec &outrec);
  void AddJoin(TEdge *e1, TEdge *e2, int e1OutIdx = -1, int e2OutIdx = -1);
  void ClearJoins();
  void AddHorzJoin(TEdge *e, int idx);
  void ClearHorzJoins();
  bool JoinPoints(const JoinRec *j, OutPt *&p1, OutPt *&p2);
  void FixupJoinRecs(JoinRec *j, OutPt *pt, unsigned startIdx);
  void JoinCommonEdges();
  void DoSimplePolygons();
  void FixupFirstLefts1(OutRec* OldOutRec, OutRec* NewOutRec);
  void FixupFirstLefts2(OutRec* OldOutRec, OutRec* NewOutRec);
};

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

class clipperException : public std::exception
{
  public:
    clipperException(const char* description): m_descr(description) {}
    virtual ~clipperException() throw() {}
    virtual const char* what() const throw() {return m_descr.c_str();}
  private:
    std::string m_descr;
};
//------------------------------------------------------------------------------

bool PointInPolygon(const IntPoint &pt, OutPt *pp, bool UseFullInt64Range);

//=========================================================================
//================== EXTENTION ============================================
//=========================================================================

void print(std::ostream& cout, std::string title, const Polygon& poly);
void print(std::ostream& cout, std::string title, const Polygons& poly);
void print(std::ostream& cout, std::string title, const PolyTree& poly);

bool operator<(const IntPoint& p, const Polygon& poly);
inline
bool PointInPolygon(const IntPoint& p, const Polygon& poly){ return p<poly; }
bool operator<(const Polygon& p1, const Polygon& p2);
inline
bool PoligonInPolygon(const Polygon& p1, const Polygon& p2){return p1<p2;}

void PolygonsToPolyTree(const Polygons& ps, PolyTree& tree);
void copy(const PolyTree& source, Polygons& dist);
void copy(const Polygons& source, PolyTree& dist);
void copy(const Polygons& source, Polygons& dist);
void copy(const PolyTree& source, PolyTree& dist);

bool operator<(const IntPoint& p, const PolyTree& poly);

void calc_union(const Polygons& p1, const Polygons& p2, Polygons& res);
void calc_intersection(const Polygons& p1, const Polygons& p2, Polygons& res);
void calc_difference(const Polygons& p1, const Polygons& p2, Polygons& res);
void calc_xor(const Polygons& p1, const Polygons& p2, Polygons& res);



} //ClipperLib namespace

#endif //clipper_hpp


