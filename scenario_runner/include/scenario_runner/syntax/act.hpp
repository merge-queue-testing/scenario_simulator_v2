#ifndef SCENARIO_RUNNER__SYNTAX__ACT_HPP_
#define SCENARIO_RUNNER__SYNTAX__ACT_HPP_

#include <scenario_runner/syntax/maneuver_group.hpp>
#include <scenario_runner/syntax/storyboard_element.hpp>

namespace scenario_runner { inline namespace syntax
{
  /* ==== Act ==================================================================
   *
   * <xsd:complexType name="Act">
   *   <xsd:sequence>
   *     <xsd:element name="ManeuverGroup" maxOccurs="unbounded" type="ManeuverGroup"/>
   *     <xsd:element name="StartTrigger" type="Trigger"/>
   *     <xsd:element name="StopTrigger" minOccurs="0" type="Trigger"/>
   *   </xsd:sequence>
   *   <xsd:attribute name="name" type="String" use="required"/>
   * </xsd:complexType>
   *
   * ======================================================================== */
  struct Act
    : public StoryboardElement<Act>
    , public Objects
  {
    const String name;

    Scope inner_scope;

    // TODO
    // - define type StartTrigger StopTrigger
    // - type StartTrigger is not default_constructible
    // - type StopTrigger is default_constructible
    // - use function 'readElement<T>' at member initialization list
    Object start_trigger, stop_trigger;

    template <typename Node>
    explicit Act(const Node& node, Scope& outer_scope)
      : name        { readAttribute<String>(node, outer_scope, "name") }
      , inner_scope { outer_scope }
    {
      callWithElements(node, "ManeuverGroup", 1, unbounded, [&](auto&& node)
      {
        return makeStoryboardElement<ManeuverGroup>(node, inner_scope);
      });

      callWithElements(node, "StartTrigger", 1, 1, [&](auto&& node)
      {
        return start_trigger.rebind<Trigger>(node, inner_scope);
      });

      callWithElements(node, "StopTrigger", 0, 1, [&](auto&& node)
      {
        return stop_trigger.rebind<Trigger>(node, inner_scope);
      });
    }

    auto ready() const
    {
      return start_trigger.evaluate().as<Boolean>();
    }

    auto stopTriggered() const
    {
      return stop_trigger and stop_trigger.evaluate().as<Boolean>();
    }

    /* -------------------------------------------------------------------------
     *
     * A ManeuverGroup's goal is accomplished when all its Maneuvers are in the
     * completeState.
     *
     * ---------------------------------------------------------------------- */
    auto accomplished() const
    {
      return std::all_of(std::begin(*this), std::end(*this), [&](auto&& each)
             {
               return each.template as<ManeuverGroup>().complete();
             });
    }

    void stop()
    {
      for (auto&& each : *this)
      {
        each.as<ManeuverGroup>().override();
        each.evaluate();
      }
    }

    using StoryboardElement::evaluate;

    void run()
    {
      for (auto&& each : *this)
      {
        each.evaluate();
      }
    }
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__ACT_HPP_
