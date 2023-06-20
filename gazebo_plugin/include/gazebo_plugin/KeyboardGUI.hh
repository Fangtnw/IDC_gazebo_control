 #define GAZEBO_PLUGINS_KEYBOARDGUIPLUGIN_HH_
 
 #include <memory>
 #include <gazebo/common/Plugin.hh>
 
 // See: https://bugreports.qt-project.org/browse/QTBUG-22829
 #ifndef Q_MOC_RUN
# include <gazebo/gui/gui.hh>
#endif
 
namespace gazebo
{
// Forward declare private data class
class KeyboardGUIPluginPrivate;
 
class GAZEBO_VISIBLE KeyboardGUIPlugin : public GUIPlugin
{
Q_OBJECT
 
public: KeyboardGUIPlugin();
 
public: virtual ~KeyboardGUIPlugin();
 
protected: void OnKeyPress(const gazebo::common::KeyEvent &_event);
 
private: bool eventFilter(QObject *_obj, QEvent *_event);
 
private: std::unique_ptr<KeyboardGUIPluginPrivate> dataPtr;
};
}
 
 #endif