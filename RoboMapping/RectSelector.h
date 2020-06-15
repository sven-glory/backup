#pragma once

class CRectSelector
{
private:
	CPoint m_pt1;
	CPoint m_pt2;
	int    m_nLeft;
	int    m_nTop;
	int    m_nRight;
	int    m_nBottom;

public:
	CRectSelector() {}

	// 设置第一个点
	void Set1stPoint(CPoint& pt1) { m_pt1 = pt1; }

	// 设置第二个点
	void Set2ndPoint(CPoint& pt2)
	{
		m_pt2 = pt2;

		m_nLeft = min(m_pt1.x, m_pt2.x);
		m_nRight = max(m_pt1.x, m_pt2.x);
		m_nTop = max(m_pt1.y, m_pt2.y);
		m_nBottom = min(m_pt1.y, m_pt2.y);
	}

	CPoint& Get1stPoint() { return m_pt1; }
	CPoint& Get2ndPoint() { return m_pt2; }

	// 判断一个点是否处于矩形区域内
	bool ContainPoint(CPoint& pt)
	{
		if (pt.x >= m_nLeft && pt.x <= m_nRight && pt.y >= m_nBottom && pt.y <= m_nTop)
			return true;
		else
			return false;
	}
};
