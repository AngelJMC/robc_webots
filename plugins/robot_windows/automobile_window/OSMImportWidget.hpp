// Copyright 1996-2018 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Description:  Tab allowing to generate worlds using the osm importer script
 */

#ifndef OSM_IMPORTER_WIDGET_HPP
#define OSM_IMPORTER_WIDGET_HPP

#include <QtWidgets/QWidget>

class QPushButton;
class QVBoxLayout;

class OSMImportWidget : public QWidget {
  Q_OBJECT

public:
  explicit OSMImportWidget(QWidget *parent = 0);
  virtual ~OSMImportWidget();

public slots:
  void launchExecutable();

protected:
  QPushButton *mPushButton;
  QVBoxLayout *mLayout;
};

#endif  // OSM_IMPORTER_WIDGET_HPP
