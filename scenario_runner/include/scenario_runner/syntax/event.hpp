#ifndef SCENARIO_RUNNER__SYNTAX__EVENT_HPP_
#define SCENARIO_RUNNER__SYNTAX__EVENT_HPP_

#include <scenario_runner/syntax/action.hpp>
#include <scenario_runner/syntax/priority.hpp>
#include <scenario_runner/syntax/storyboard_element.hpp>
#include <scenario_runner/syntax/trigger.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== Event ================================================================
   *
   * <xsd:complexType name="Event">
   *   <xsd:sequence>
   *     <xsd:element name="Action" maxOccurs="unbounded" type="Action"/>
   *     <xsd:element name="StartTrigger" type="Trigger"/>
   *   </xsd:sequence>
   *   <xsd:attribute name="priority" type="Priority" use="required"/>
   *   <xsd:attribute name="maximumExecutionCount" type="UnsignedInt"/>
   *   <xsd:attribute name="name" type="String" use="required"/>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct Event
    : public StoryboardElement<Event>
    , public Objects
  {
    // Name of the event.
    const String name;

    // Priority of each event.
    const Priority priority;

    Scope inner_scope;

    Object start_trigger;

    template <typename Node, typename Scope>
    explicit Event(const Node& node, Scope& outer_scope)
      : StoryboardElement { readAttribute<UnsignedInt>(node, outer_scope, "maximumExecutionCount", 1) }
      , name              { readAttribute<String>     (node, outer_scope, "name") }
      , priority          { readAttribute<Priority>   (node, outer_scope, "priority") }
      , inner_scope       { outer_scope }
    {
      callWithElements(node, "Action", 1, unbounded, [&](auto&& node)
      {
        return makeStoryboardElement<Action>(node, inner_scope);
      });

      callWithElements(node, "StartTrigger", 1, 1, [&](auto&& node)
      {
        return start_trigger.rebind<Trigger>(node, inner_scope);
      });
    }

    auto ready() const
    {
      return start_trigger.evaluate().as<Boolean>(__FILE__, __LINE__);
    }

    static constexpr auto stopTriggered() noexcept
    {
      return false;
    }

    /* -------------------------------------------------------------------------
     *
     * Event
     *   An Event's goal is accomplished when all its Actions are in the
     *   completeState.
     *
     * ---------------------------------------------------------------------- */
    auto accomplished() const
    {
      return std::all_of(std::begin(*this), std::end(*this), [](auto&& each)
             {
               return each.template as<Action>().complete();
             });
    }

    using StoryboardElement::evaluate;

    void stop()
    {
      for (auto&& each : *this)
      {
        each.as<Action>().override();
        each.evaluate();
      }
    }

    void run()
    {
      for (auto&& each : *this)
      {
        each.evaluate();
      }
    }
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__EVENT_HPP_
