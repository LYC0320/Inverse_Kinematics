#ifndef _GUI_INVERSE_KINEMATICS_MAIN_WINDOW_H_
#define _GUI_INVERSE_KINEMATICS_MAIN_WINDOW_H_

#pragma warning(disable:4800)
#include "gui_def.h"
#include <memory>
#include <utility>
#include <map>
#include <functional>
#include "boost/ptr_container/ptr_container.hpp"
#include "boost/optional.hpp"
#include "FL/Fl.H"
#include "FL/Fl_Window.H"
#include "FL/Fl_Widget.H"
#include "param_fwd.h"
#include "math_type.h"
#include "gui_fwd.h"
#include "gui_fl_fwd.h"
#include "gui_type.h"
#include "gui_fl_widget_callback_data.h"

namespace gui {

class InverseKinematicsMainWindow final : public Fl_Window
{

public:
    /**
     * \brief
     */
    InverseKinematicsMainWindow(
            const int32_t width,
            const int32_t height,
            const char *title
            );
    /**
     * \brief
     */
    InverseKinematicsMainWindow(const InverseKinematicsMainWindow &) = delete;
    /**
     * \brief
     */
    virtual ~InverseKinematicsMainWindow();
    /**
     * \brief
     */
    InverseKinematicsMainWindow &operator=(const InverseKinematicsMainWindow &) = delete;
    /**
     * \brief
     */
    virtual int32_t handle(int32_t event) override;

    void Configure(
            const std::shared_ptr<param::Config> &param,
            const std::shared_ptr<Signal> &signal
            );

protected:

private:

    typedef struct TimeoutCallbackData
    {
        std::shared_ptr<Signal> signal;
        boost::optional<const double &> time_step;
    } TimeoutCallbackData_t;

    typedef boost::ptr_map<const std::string, Fl_Button> ButtonMap_t;
    typedef boost::ptr_map<const std::string, Fl_Light_Button> LightButtonMap_t;
    typedef boost::ptr_map<const std::string, Fl_Value_Input> ValueInputMap_t;
    typedef boost::ptr_map<const std::string, Fl_Value_Slider> ValueSliderMap_t;

    typedef boost::ptr_map<const std::string, FlWidgetCallbackData_t> WidgetCallbackDataMap_t;
    typedef boost::function<void (InverseKinematicsMainWindow *, Fl_Widget *)> SlotFunc_t;
    typedef std::map<std::string, SlotFunc_t> SlotFuncMap_t;

    /**
     * \brief Initialize the button (Fl_Button) on the main window
     */
    void InitButton();
    /**
     * \brief Initialize the light button (Fl_Light_Button) on the main window
     */
    void InitLightButton();
    /**
     * \brief Initialize the value input (Fl_Value_Input) on the main window
     */
    void InitValueInput();
    /**
     * \brief Initialize the value slider (Fl_Value_Slider) on the main window
     */
    void InitValueSlider();
    /**
     * \brief Initialized the GL window for rendering
     */
    void InitGlWindow();
    /**
     * \brief
     */
    void DeactivateWidget();
    /**
     * \brief Set the correspdongin switch, specified by id, value to is_on
     */
    void set_switch(const Id_t &id, const bool is_on);
    /**
     * \brief Get the value of the switch, specified by id
     */
    bool is_switch_on(const Id_t &id);
    /**
     * \brief Callback (slot) function of the "Exit" button (signal)
     */
    void ExitSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "Load motion" button (signal)
     */
    void LoadMotionSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "Load skeleton" button (signal)
     */
    void LoadSkeletonSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "Reset skeleton" button (signal)
     */
    void ResetSkeletonSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "Reset marker" button (signal)
     */
    void ResetMarkerSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "Slerp" button (signal)
     */
    void CleanFootskateSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "Save" button (signal)
     */
    void SaveSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "@|<" (rewind) button (signal)
     */
    void RewindSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "@||" (pause) button (signal)
     */
    void PauseSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "@>" (play) button (signal)
     */
    void PlaySlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "@<->" (repeat) button (signal)
     */
    void RepeatSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "R" (record) light button (signal)
     */
    void RecordSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "Light" light button (singal)
     */
    void LightSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "Ground" light button (signal)
     */
    void GroundSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "IK marker" light button (signal)
     */
    void IkMarkerSlot(Fl_Widget *widget);
    /**
     * \brief
     */
    //void IkTrajectorySlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "IK mode" light button (signal)
     */
    void IkModeSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "frameSlider" Fl_Value_Slider (signal)
     */
    void FrameSlideSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "Frame steps" Fl_Value_Input (signal)
     */
    void FrameStepsSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "dt" Fl_Value_Input (signal)
     */
    void FrameOffsetSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "sub" Fl_Value_Input (signal)
     */
    void IndicateSkeletonSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "tx" Fl_Value_Input (signal)
     */
    void SkeletonTranslationXSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "ty" Fl_Value_Input (signal)
     */
    void SkeletonTranslationYSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "tz" Fl_Value_Input (signal)
     */
    void SkeletonTranslationZSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "rx" Fl_Value_Input (signal)
     */
    void SkeletonRotationXSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "ry" Fl_Value_Input (signal)
     */
    void SkeletonRotationYSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "rz" Fl_Value_Input (signal)
     */
    void SkeletonRotationZSlot(Fl_Widget *widget);
    /**
     * \brief Callback (slot) function of the "Joint" Fl_Value_Input (signal)
     */
    void JointIdxSlot(Fl_Widget *widget);
    /**
     * \brief Callback function when timeout at each forward step
     */
    void TimeoutSlot();
    void IkTimeoutSlot();

    static void WidgetTimeoutCallback(void *data);
    /**
     * \brief
     */
    math::Vector6d_t skeleton_root_offset() const;

    std::unique_ptr<ButtonMap_t> button_map_;
    std::unique_ptr<LightButtonMap_t> light_button_map_;
    std::unique_ptr<ValueInputMap_t> value_input_map_;
    std::unique_ptr<ValueSliderMap_t> value_slider_map_;

    std::unique_ptr<InverseKinematicsGlWindow> gl_window_;
    std::unique_ptr<InverseKinematicsDisplay> display_;
    std::shared_ptr<Renderer> renderer_;

    std::unique_ptr<SlotFuncMap_t> button_slot_map_;
    std::unique_ptr<SlotFuncMap_t> light_button_slot_map_;
    std::unique_ptr<SlotFuncMap_t> value_input_slot_map_;
    std::unique_ptr<SwitchMap_t> switch_map_;
    std::unique_ptr<Mouse> mouse_;

    std::shared_ptr<Marker> ik_marker_;

    std::shared_ptr<param::Config> param_;
    std::shared_ptr<Signal> signal_;

    int32_t max_frame_num_;
    int32_t frame_idx_;
    int32_t frame_increment_;
    double time_step_;

    std::unique_ptr<TimeoutCallbackData_t> timeout_callback_data_;

    std::vector<std::shared_ptr<FlWidgetCallbackData_t>> callback_data_coll_;
    std::unique_ptr<WidgetCallbackDataMap_t> widget_callback_data_map_;

    boost::signals2::connection timeout_connection_;
};

} // namespace gui {

#endif // #ifndef _GUI_INVERSE_KINEMATICS_MAIN_WINDOW_H_
