#ifndef __ddViewManager_h
#define __ddViewManager_h

#include <QWidget>
#include "ddAppConfigure.h"

class QTabWidget;
class ddViewBase;

class DD_APP_EXPORT ddViewManager : public QWidget
{
    Q_OBJECT

public:

  ddViewManager(QWidget* parent=0);
  virtual ~ddViewManager();

  QTabWidget* tabWidget() const;

  ddViewBase* findView(const QString& viewName, const QString& robotName="") const;

  ddViewBase* createView(const QString& viewName, const QString& viewType, int pageIndex=-1, const QString& robotName="");

  void hideView(ddViewBase* view, bool storeLocation=true);

  void showView(ddViewBase* view);

  void switchToView(const QString& viewName);

  ddViewBase* currentView() const;

  std::pair<QString, QString> viewName(ddViewBase* view);

  void popOut(ddViewBase* view);

  void updatePageIndexCache();

signals:

  void currentViewChanged(ddViewBase* previousView, ddViewBase* currentView);

protected slots:

  void onCurrentTabChanged(int currentIndex);

protected:

  bool eventFilter(QObject* obj, QEvent* event);

  void addDefaultPage();

  void addView(ddViewBase* view, const QString& viewName, int pageIndex=-1, const QString& robotName="");

  class ddInternal;
  ddInternal* Internal;

  Q_DISABLE_COPY(ddViewManager);


};

#endif
